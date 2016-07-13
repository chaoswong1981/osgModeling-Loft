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

#ifndef OSGMODELING_TEXCOORDVISITOR
#define OSGMODELING_TEXCOORDVISITOR 1

#include <vEngine/NodeVisitor.h>
#include <vEngine/Geode.h>
#include <vEngine/Geometry.h>
#include "Curve.h"

namespace osgModeling {

class Model;

/** Texture coordinates creating visitor class
 * It supports creating texture coordinates and binding to different planes.
 */
class TexCoordVisitor : public vEngine::NodeVisitor
{
public:
    TexCoordVisitor();
    virtual ~TexCoordVisitor();

    /** Create texture coordinates for geometry. */
    static void buildTexCoord( vEngine::Geometry& geoset );

    virtual void apply( vEngine::Geode& geode );
};

}

#endif
