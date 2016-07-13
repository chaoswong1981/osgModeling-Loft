/* -*-c++-*- osgModeling - Copyright (C) 2008 Wang Rui <wangray84@gmail.com>
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.

* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.

* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include "Curve.h"

using namespace osgModeling;

Curve::Curve():
    vEngine::Object(),
    _pathPts(0), _algorithmCallback(0), _updated(false)
{
}

Curve::Curve( const Curve& copy, const vEngine::CopyOp& copyop/*=vEngine::CopyOp::SHALLOW_COPY*/ ):
    vEngine::Object(copy,copyop),
    _algorithmCallback(copy._algorithmCallback), _updated(copy._updated)
{
    _pathPts = dynamic_cast<vEngine::Vec3Array*>( copy._pathPts->clone(copyop) );
}

Curve::~Curve()
{
}

vEngine::Vec3 Curve::mapTo( const vEngine::Vec3 p, vEngine::BoundingBox originRect, vEngine::BoundingBox newRect )
{
    vEngine::Vec3 newPos = p-originRect.center();

    vEngine::Vec3 originSpace = originRect._max-originRect._min;
    vEngine::Vec3 scaleFactor = (newRect._max-newRect._min);
    if ( originSpace.x() ) scaleFactor.x() /= originSpace.x();
    else scaleFactor.x() = 0.0;
    if ( originSpace.y() ) scaleFactor.y() /= originSpace.y();
    else scaleFactor.y() = 0.0;
    if ( originSpace.z() ) scaleFactor.z() /= originSpace.z();
    else scaleFactor.z() = 0.0;

    newPos = vEngine::Vec3(newPos.x()*scaleFactor.x(), newPos.y()*scaleFactor.y(), newPos.z()*scaleFactor.z());
    return newPos+newRect.center();
}

vEngine::Vec2 Curve::mapTo2D( const vEngine::Vec3 p, vEngine::BoundingBox originRect, vEngine::BoundingBox newRect )
{
    if ( originRect.xMin()==originRect.xMax() )
    {
        newRect.zMin() = newRect.yMin(); newRect.zMax() = newRect.yMax();
        newRect.yMin() = newRect.xMin(); newRect.yMax() = newRect.xMax();
        newRect.xMin() = 0.0;            newRect.xMax() = 0.0;
    }
    else if ( originRect.yMin()==originRect.yMax() )
    {
        newRect.zMin() = newRect.yMin(); newRect.zMax() = newRect.yMax();
        newRect.yMin() = 0.0;            newRect.yMax() = 0.0;
    }

    vEngine::Vec3 newPoint = mapTo( p, originRect, newRect );
    vEngine::Vec2 point2D( newPoint.x(), newPoint.y() );
    if ( originRect.xMin()==originRect.xMax() ) point2D.set( newPoint.y(), newPoint.z() );
    else if ( originRect.yMin()==originRect.yMax() ) point2D.set( newPoint.x(), newPoint.z() );

    return point2D;
}
