
/* Copyright (c) 2011-2012, Stefan Eilemann <eile@eyescale.ch>
 *                    2012, Daniel Nachbaur <danielnachbaur@gmail.com>
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

#include "viewData.h"

#ifndef EQ_2_0_API
#  include <eq/client/configEvent.h>
#endif
#include <eq/client/event.h>
#include <eq/client/eventICommand.h>
#include <co/dataIStream.h>
#include <co/dataOStream.h>

namespace seq
{
ViewData::ViewData()
    : _modelMatrix( eq::Matrix4f::IDENTITY )
    , _pivotPoint( eq::Vector3f::ZERO )
    , _modelRadius( 1.f )
    , _spinX( 5 )
    , _spinY( 5 )
    , _advance( 0 )
    , _statistics( false )
    , _ortho( false )
{
    resetModelPosition();
}

ViewData::~ViewData()
{}

void ViewData::serialize( co::DataOStream& os, const uint64_t dirtyBits )
{
    co::Serializable::serialize( os, dirtyBits );
    if( dirtyBits & DIRTY_MODELMATRIX )
        os << _modelMatrix;
    if( dirtyBits & DIRTY_STATISTICS )
        os << _statistics;
    if( dirtyBits & DIRTY_ORTHO )
        os << _ortho;
}

void ViewData::deserialize( co::DataIStream& is, const uint64_t dirtyBits )
{
    co::Serializable::deserialize( is, dirtyBits );
    if( dirtyBits & DIRTY_MODELMATRIX )
        is >> _modelMatrix;
    if( dirtyBits & DIRTY_STATISTICS )
        is >> _statistics;
    if( dirtyBits & DIRTY_ORTHO )
        is >> _ortho;
}
#ifndef EQ_2_0_API
bool ViewData::handleEvent( const eq::ConfigEvent* event )
{
    return _handleEvent( event->data );
}
#endif

bool ViewData::handleEvent( eq::EventICommand command )
{
    const eq::Event& event = command.get< eq::Event >();
    return _handleEvent( event );
}

bool ViewData::_handleEvent( const eq::Event& event )
{
    const float magellanWeight = 0.0001f * _modelRadius;
    const float magellanSpinWeight = 0.0001f;
    const float mouseSpinWeight = 0.005f;
    const float mousePanWeight = 0.0005f * _modelRadius;
    const float mouseZoomWeight = 0.005f * _modelRadius;

    switch( event.type )
    {
      case eq::Event::CHANNEL_POINTER_BUTTON_RELEASE:
      {
          const eq::PointerEvent& releaseEvent =
              event.pointerButtonRelease;
          if( releaseEvent.buttons == eq::PTR_BUTTON_NONE )
          {
              if( releaseEvent.button == eq::PTR_BUTTON1 )
              {
                  _spinX = releaseEvent.dy;
                  _spinY = releaseEvent.dx;
                  return true;
              }
              if( releaseEvent.button == eq::PTR_BUTTON2 )
              {
                  _advance = -releaseEvent.dy;
                  return true;
              }
          }
          return false;
      }
      case eq::Event::CHANNEL_POINTER_MOTION:
          switch( event.pointerMotion.buttons )
          {
            case eq::PTR_BUTTON1:
                _spinX = 0;
                _spinY = 0;
                spinModel( -mouseSpinWeight * event.pointerMotion.dy,
                           -mouseSpinWeight * event.pointerMotion.dx, 0.f );
                return true;

            case eq::PTR_BUTTON2:
                _advance = -event.pointerMotion.dy;
                _moveModelScaledToZ( 0.f, 0.f, mouseZoomWeight * _advance );
                return true;

            case eq::PTR_BUTTON3:
                _moveModelScaledToZ(
                     mousePanWeight * event.pointerMotion.dx,
                    -mousePanWeight * event.pointerMotion.dy, 0.f );
                return true;

            default:
                return false;
          }

      case eq::Event::CHANNEL_POINTER_WHEEL:
          _moveModelScaledToZ( -mouseZoomWeight * event.pointerWheel.yAxis, 0.f,
                                mouseZoomWeight * event.pointerWheel.xAxis );
          return true;

      case eq::Event::MAGELLAN_AXIS:
          _spinX = 0;
          _spinY = 0;
          _advance = 0;
          spinModel(  magellanSpinWeight * event.magellan.zRotation,
                     -magellanSpinWeight * event.magellan.xRotation,
                     -magellanSpinWeight * event.magellan.yRotation );
          _moveModelScaledToZ(  magellanWeight * event.magellan.xAxis,
                               -magellanWeight * event.magellan.zAxis,
                                magellanWeight * event.magellan.yAxis );
          return true;

      case eq::Event::KEY_PRESS:
          switch( event.keyPress.key )
          {
            case 's':
                showStatistics( !getStatistics( ));
                return true;
            case 'o':
                setOrtho( !useOrtho( ));
                return true;
          }
          return false;

      default:
          return false;
    }
}

void ViewData::spinModel( const float x, const float y, const float z )
{
    if( x == 0.f && y == 0.f && z == 0.f )
        return;

    Vector3f translation;
    _modelMatrix.get_translation( translation );

    // Recovering the translation without the rotation around the pivot.
    _modelMatrix.set_translation( Vector3f::ZERO );
    translation -= _pivotPoint;
    translation -= _modelMatrix * -_pivotPoint;

    // Finding the new rotation
    _modelMatrix.set_translation( Vector3f::ZERO );
    _modelMatrix.pre_rotate_x( x );
    _modelMatrix.pre_rotate_y( y );
    _modelMatrix.pre_rotate_z( z );

    // Composing the translation part of the new _modelMatrix.
    _modelMatrix.set_translation(
        _modelMatrix * -_pivotPoint + translation + _pivotPoint);

    setDirty( DIRTY_MODELMATRIX );
}

void ViewData::moveModel( const float x, const float y, const float z )
{
    if( x == 0.f && y == 0.f && z == 0.f )
        return;

    Vector3f translation;
    _modelMatrix.get_translation( translation );
    _modelMatrix.set_translation( translation + Vector3f( x, y, z ));
    setDirty( DIRTY_MODELMATRIX );
}

void ViewData::setModelBounding( const Vector3f& center, const float radius )
{
    _pivotPoint = center;
    _modelRadius = radius;
}

void ViewData::resetModelPosition()
{
    // Assuming a field of view of 45 degrees.
    float distance = _modelRadius * 1.5 / std::sin(M_PI / 4);
    _modelMatrix.set_translation(
        Vector3f( -_pivotPoint[0], -_pivotPoint[1], -distance ));
}

void ViewData::showStatistics( const bool on )
{
    if( _statistics == on )
        return;

    _statistics = on;
    setDirty( DIRTY_STATISTICS );
}

void  ViewData::setOrtho( const bool on )
{
    if( _ortho == on )
        return;

    _ortho = on;
    setDirty( DIRTY_ORTHO );
}

bool ViewData::update()
{
    if( _spinX == 0 && _spinY == 0 && _advance == 0 )
        return false;

    spinModel( -0.001f * _spinX, -0.001f * _spinY, 0.f );
    _moveModelScaledToZ( 0.0f, 0.0f, 0.001f * _advance );
    return true;
}

void ViewData::_moveModelScaledToZ( const float x, const float y,
                                    const float z )
{
    if( x == 0.f && y == 0.f && z == 0.f )
        return;
    Vector3f translation;
    _modelMatrix.get_translation( translation );
    translation += _pivotPoint;

    // Recovering the distance along Z without the rotation around the pivot.
    float distance = translation[2] - _pivotPoint[2] +
        dot( _modelMatrix.get_row( 2 ), eq::Vector4f( _pivotPoint, 1 ));

    const float scaling = std::abs(distance) / _modelRadius;
    translation += Vector3f( x, y, z) * scaling;

    translation -= _pivotPoint;
    _modelMatrix.set_translation( translation );
    setDirty( DIRTY_MODELMATRIX );
}

}

