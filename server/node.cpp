
/* Copyright (c) 2005-2007, Stefan Eilemann <eile@equalizergraphics.com> 
   All rights reserved. */

#include <pthread.h>
#include "node.h"

#include "channel.h"
#include "config.h"
#include "global.h"
#include "log.h"
#include "pipe.h"
#include "server.h"
#include "window.h"

#include <eq/net/barrier.h>
#include <eq/net/command.h>
#include <eq/client/packets.h>

using namespace eqs;
using namespace eqBase;
using namespace std;

typedef std::vector<Pipe*>::const_iterator PipeIter;

void Node::_construct()
{
    _used             = 0;
    _config           = NULL;
    _lastDrawCompound = 0;

    registerCommand( eq::CMD_NODE_CONFIG_INIT_REPLY, 
                  eqNet::CommandFunc<Node>( this, &Node::_cmdConfigInitReply ));
    registerCommand( eq::CMD_NODE_CONFIG_EXIT_REPLY, 
                  eqNet::CommandFunc<Node>( this, &Node::_cmdConfigExitReply ));
    registerCommand( eq::CMD_NODE_FRAME_FINISH_REPLY,
                 eqNet::CommandFunc<Node>( this, &Node::_cmdFrameFinishReply ));

    EQINFO << "Add node @" << (void*)this << endl;
}

Node::Node()
{
    _construct();
}

Node::Node( const Node& from )
        : eqNet::Object()
{
    _construct();

    _name = from._name;
    _node = from._node;

    const uint32_t numConnectionDescriptions = from.nConnectionDescriptions();
    for( uint32_t i=0; i<numConnectionDescriptions; i++ )
    {
        eqNet::ConnectionDescription* desc = 
            from.getConnectionDescription(i).get();
        
        addConnectionDescription( new eqNet::ConnectionDescription( *desc ));
    }

    const uint32_t numPipes = from.nPipes();
    for( uint32_t i=0; i<numPipes; i++ )
    {
        Pipe* pipe      = from.getPipe(i);
        Pipe* pipeClone = new Pipe( *pipe );
            
        addPipe( pipeClone );
    }
}

Node::~Node()
{
    EQINFO << "Delete node @" << (void*)this << endl;

    if( _config )
        _config->removeNode( this );
    
    for( vector<Pipe*>::const_iterator i = _pipes.begin(); i != _pipes.end();
         ++i )
    {
        Pipe* pipe = *i;

        pipe->_node = 0;
        delete pipe;
    }
    _pipes.clear();
}

void Node::addPipe( Pipe* pipe )
{
    _pipes.push_back( pipe ); 
    pipe->_node = this; 
}

bool Node::removePipe( Pipe* pipe )
{
    vector<Pipe*>::iterator i = find( _pipes.begin(), _pipes.end(), pipe );
    if( i == _pipes.end( ))
        return false;

    _pipes.erase( i );
    pipe->_node = 0; 

    return true;
}

//===========================================================================
// Operations
//===========================================================================

//---------------------------------------------------------------------------
// init
//---------------------------------------------------------------------------
void Node::startConfigInit( const uint32_t initID )
{
    EQASSERT( _state == STATE_STOPPED );
    _state = STATE_INITIALIZING;

    eq::NodeConfigInitPacket packet;
    packet.initID = initID;
    _send( packet, _name );
    EQLOG( eq::LOG_TASKS ) << "TASK pipe configInit  " << &packet << endl;

    eq::NodeCreatePipePacket createPipePacket;
    for( PipeIter i = _pipes.begin(); i != _pipes.end(); ++i )
    {
        Pipe* pipe = *i;
        if( !pipe->isUsed( ))
            continue;
        
        _config->registerObject( pipe );
        createPipePacket.pipeID = pipe->getID();
        _send( createPipePacket );
        pipe->startConfigInit( initID );
    }

    _bufferedTasks.sendBuffer( _node->getConnection( ));
}

bool Node::syncConfigInit()
{
    bool success = true;
    for( PipeIter i = _pipes.begin(); i != _pipes.end(); ++i )
    {
        Pipe* pipe = *i;
        if( !pipe->isUsed( ))
            continue;

        if( !pipe->syncConfigInit( ))
        {
            _error += "pipe: '" + pipe->getErrorMessage() + '\'';
            success = false;
        }
    }

    EQASSERT( _state == STATE_INITIALIZING || _state == STATE_RUNNING ||
              _state == STATE_INIT_FAILED );
    _state.waitNE( STATE_INITIALIZING );
    if( _state == STATE_INIT_FAILED )
        success = false;

    if( !success )
        EQWARN << "Node initialisation failed: " << _error << endl;
    return success;
}

//---------------------------------------------------------------------------
// exit
//---------------------------------------------------------------------------
void Node::startConfigExit()
{
    EQASSERT( _state == STATE_RUNNING || _state == STATE_INIT_FAILED );
    _state = STATE_STOPPING;

    for( PipeIter i = _pipes.begin(); i != _pipes.end(); ++i )
    {
        Pipe* pipe = *i;

        if( pipe->getState() != Pipe::STATE_STOPPED )
            pipe->startConfigExit();
    }

    eq::NodeConfigExitPacket packet;
    _send( packet );
    _bufferedTasks.sendBuffer( _node->getConnection( ));
}

bool Node::syncConfigExit()
{
    EQASSERT( _state == STATE_STOPPING || _state == STATE_STOPPED || 
              _state == STATE_STOP_FAILED );
    
    _state.waitNE( STATE_STOPPING );
    bool success = ( _state == STATE_STOPPED );
    EQASSERT( success || _state == STATE_STOP_FAILED );
    _state = STATE_STOPPED; // STOP_FAILED -> STOPPED transition

    eq::NodeDestroyPipePacket destroyPipePacket;
    for( PipeIter i = _pipes.begin(); i != _pipes.end(); ++i )
    {
        Pipe* pipe = *i;
        if( pipe->getID() == EQ_ID_INVALID )
            continue;

        if( !pipe->syncConfigExit( ))
        {
            EQWARN << "Could not exit cleanly: " << pipe << endl;
            success = false;
        }

        destroyPipePacket.pipeID = pipe->getID();
        _send( destroyPipePacket );
        _config->deregisterObject( pipe );
    }
    _bufferedTasks.sendBuffer( _node->getConnection( ));

    _flushBarriers();
    return success;
}

//---------------------------------------------------------------------------
// update
//---------------------------------------------------------------------------
void Node::update( const uint32_t frameID, const uint32_t frameNumber )
{
    EQVERB << "Start frame" << endl;

    if( !_lastDrawCompound )
    {
        Config* config = getConfig();
        _lastDrawCompound = config->getCompound(0);
    }

    eq::NodeFrameStartPacket startPacket;
    startPacket.frameID     = frameID;
    startPacket.frameNumber = frameNumber;
    _send( startPacket );
    EQLOG( eq::LOG_TASKS ) << "TASK node start frame " << &startPacket << endl;

    for( vector< Pipe* >::const_iterator i = _pipes.begin();
         i != _pipes.end(); ++i )
    {
        Pipe* pipe = *i;
        if( pipe->isUsed( ))
            pipe->update( frameID, frameNumber );
    }

    eq::NodeFrameFinishPacket finishPacket;
    finishPacket.frameID     = frameID;
    finishPacket.frameNumber = frameNumber;
    _send( finishPacket );
    EQLOG( eq::LOG_TASKS ) << "TASK node finish frame  " << &finishPacket
                           << endl;

    _bufferedTasks.sendBuffer( _node->getConnection( ));
    _lastDrawCompound = 0;
}

//---------------------------------------------------------------------------
// Barrier cache
//---------------------------------------------------------------------------
eqNet::Barrier* Node::getBarrier()
{
    if( _barriers.empty() )
    {
        eqNet::Barrier* barrier = new eqNet::Barrier( _node );
        _config->registerObject( barrier );
        barrier->setAutoObsolete( getConfig()->getLatency()+1, 
                                  Object::AUTO_OBSOLETE_COUNT_VERSIONS );
        
        return barrier;
    }

    eqNet::Barrier* barrier = _barriers.back();
    _barriers.pop_back();
    barrier->setHeight(0);
    return barrier;
}

void Node::releaseBarrier( eqNet::Barrier* barrier )
{
    _barriers.push_back( barrier );
}

void Node::_flushBarriers()
{
    for( vector< eqNet::Barrier* >::const_iterator i =_barriers.begin(); 
         i != _barriers.end(); ++ i )
    {
        eqNet::Barrier* barrier = *i;
        _config->deregisterObject( barrier );
        delete barrier;
    }
    _barriers.clear();
}

//===========================================================================
// command handling
//===========================================================================
eqNet::CommandResult Node::_cmdConfigInitReply( eqNet::Command& command )
{
    const eq::NodeConfigInitReplyPacket* packet = 
        command.getPacket<eq::NodeConfigInitReplyPacket>();
    EQINFO << "handle configInit reply " << packet << endl;

    _error = packet->error;
    if( packet->result )
        _state = STATE_RUNNING;
    else
        _state = STATE_INIT_FAILED;

    return eqNet::COMMAND_HANDLED;
}

eqNet::CommandResult Node::_cmdConfigExitReply( eqNet::Command& command )
{
    const eq::NodeConfigExitReplyPacket* packet =
        command.getPacket<eq::NodeConfigExitReplyPacket>();
    EQINFO << "handle configExit reply " << packet << endl;

    if( packet->result )
        _state = STATE_STOPPED;
    else
        _state = STATE_STOP_FAILED;

    return eqNet::COMMAND_HANDLED;
}

eqNet::CommandResult Node::_cmdFrameFinishReply( eqNet::Command& command )
{
    const eq::NodeFrameFinishReplyPacket* packet = 
        command.getPacket<eq::NodeFrameFinishReplyPacket>();
    EQVERB << "handle frame finish reply " << packet << endl;
    
    // Move me
    for( uint32_t i =0; i<packet->nStatEvents; ++i )
    {
        const eq::StatEvent::Data& data = packet->statEvents[i];
        EQLOG( LOG_STATS ) << data << endl;
    }

    _finishedFrame = packet->frameNumber;
    return eqNet::COMMAND_HANDLED;
}

ostream& eqs::operator << ( ostream& os, const Node* node )
{
    if( !node )
        return os;
    
    os << disableFlush;
    const Config* config = node->getConfig();
    if( config && config->isApplicationNode( node ))
        os << "appNode" << endl;
    else
        os << "node" << endl;

    os << "{" << endl << indent;

    const std::string& name = node->getName();
    if( !name.empty( ))
        os << "name     \"" << name << "\"" << endl;

    const uint32_t nConnectionDescriptions = node->nConnectionDescriptions();
    for( uint32_t i=0; i<nConnectionDescriptions; i++ )
        os << node->getConnectionDescription( i ).get();

    const uint32_t nPipes = node->nPipes();
    for( uint32_t i=0; i<nPipes; i++ )
        os << node->getPipe(i);

    os << exdent << "}" << enableFlush << endl;
    return os;
}
