
/* Copyright (c) 2006-2007, Stefan Eilemann <eile@equalizergraphics.com> 
 *
 * This library is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License version 2.1 as published
 * by the Free Software Foundation.
 *  
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 * details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <test.h>

#include <eq/net/barrier.h>
#include <eq/net/connection.h>
#include <eq/net/init.h>
#include <eq/net/node.h>
#include <eq/net/session.h>

#include <iostream>

using namespace eq::base;
using namespace eq::net;
using namespace std;

uint32_t barrierID = EQ_ID_INVALID;

class NodeThread : public Thread
{
public:
    NodeThread( const bool master ) : _master(master) {}

    virtual void* run()
        {
            ConnectionDescriptionPtr description = new ConnectionDescription;
            description->type       = CONNECTIONTYPE_TCPIP;
            description->TCPIP.port = _master ? 4242 : 4243;

            ConnectionPtr connection = Connection::create( description );

            TEST( connection->listen( ))
            
            RefPtr<Node> node = new Node();
            TEST( node->listen( connection ));

            if( _master )
            {
                Session session;
                TEST( node->mapSession( node, &session, "foo" ));
                
                Barrier barrier( node, 2 );
                session.registerObject( &barrier );
                TEST( barrier.getID() != EQ_ID_INVALID );
                
                barrierID = barrier.getID();

                cerr << "Master enter" << endl;
                barrier.enter();
                cerr << "Master left" << endl;

                //session.deregisterObject( &barrier );
                //node->unmapSession( &session );
            }
            else
            {
                while( barrierID == EQ_ID_INVALID );

                RefPtr<Node>                  server     = new Node;
                RefPtr<ConnectionDescription> serverDesc = 
                    new ConnectionDescription;
                serverDesc->TCPIP.port = 4242;
                server->addConnectionDescription( serverDesc );

                Session session;
                TEST( node->mapSession( server, &session, "foo" ));
                
                RefPtr<eq::net::Object> object = session.getObject( barrierID);
                TEST( dynamic_cast<eq::net::Barrier*>(object.get()) );
                
                eq::net::Barrier* barrier = (eq::net::Barrier*)object.get();
                TEST( barrier );

                cerr << "Slave enter" << endl;
                barrier->enter();
                cerr << "Slave left" << endl;

                //session.deregisterObject( barrier );
                //node->unmapSession( &session );
            }

            node->stopListening();
            return EXIT_SUCCESS;
        }
            
private:
    bool _master;
};

int main( int argc, char **argv )
{
    eq::net::init( argc, argv );

    NodeThread server( true );
    NodeThread node( false );

    server.start();
    node.start();
    server.join();
    node.join();
}

