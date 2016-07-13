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

#ifndef OSGMODELING_CURVE
#define OSGMODELING_CURVE 1

#include <vEngine/Object.h>
#include <vEngine/Array.h>
#include <vEngine/BoundingBox.h>
#include "Algorithm.h"

namespace osgModeling {

/** Curve base class
 * This is the base class of all curves.
 */
class Curve : public vEngine::Object
{
public:
    Curve();
    Curve( const Curve& copy, const vEngine::CopyOp& copyop=vEngine::CopyOp::SHALLOW_COPY );

    META_Object( osgModeling, Curve );

    /** Add a new point to the path. */
    inline void addPathPoint( vEngine::Vec3 v )
    {
        if ( !_pathPts ) _pathPts = new vEngine::Vec3Array;
        _pathPts->push_back(v);
    }

    /** Specifies a vertex list as the curve path. */
    inline void setPath( vEngine::Vec3Array* pts ) { _pathPts=pts; }
    inline vEngine::Vec3Array* getPath() { return _pathPts.get(); }
    inline const vEngine::Vec3Array* getPath() const { return _pathPts.get(); }

    /** Specifies a pointer to create the curve path. Provided for convenience.
     * Note that the parameter 'size' means amount of values, not number of vertices.
     */
    inline void setPath( unsigned int size, double* ptr )
    {
        if ( !size || !ptr ) return;
        if ( !_pathPts ) _pathPts = new vEngine::Vec3Array;
        else _pathPts->clear();

        for ( unsigned int i=0; i<size; i+=3 )
        {
            _pathPts->push_back( vEngine::Vec3(
                *(ptr+i), *(ptr+i+1), *(ptr+i+2)) );
        }
    }

    /** Set the curve generating algorithm to use.
     * Every inherited curve class has a default algorithm to create its path points (points on the curve path).
     * User may easily inherit AlgorithmCallback to realize better algorithms, and set it to the curve class.
     */
    inline void setAlgorithmCallback( AlgorithmCallback* ac ) { _algorithmCallback=ac; }
    inline AlgorithmCallback* getAlgorithmCallback() { return _algorithmCallback.get(); }

    /** Check if the curve is closed, which means the last curve point equals with the first. */
    inline bool isClosed() { return _pathPts->back()==_pathPts->front(); }

    /** Call this before drawing to generate primitives.
    * If need to be modified while running, the object should set to DYNAMIC.
    * \param forceUpdate Set to true to force rebuilding, otherwise the function may be ignored because nothing changed.
    */
    virtual void update( bool forceUpdate=false )
    {
        if ( _updated && !forceUpdate )
            return;

        if ( _algorithmCallback.valid() )
            (*_algorithmCallback)( this );
        else
            updateImplementation();

        _updated = true;
    }

    virtual void updateImplementation() {}

    /** Map a point in an box to another, scaling to fit the new one. */
    static vEngine::Vec3 mapTo( const vEngine::Vec3 p, vEngine::BoundingBox originRect, vEngine::BoundingBox newRect );

    /** Map a point in an plane to another plane, scaling and switching to fit the new one. */
    static vEngine::Vec2 mapTo2D( const vEngine::Vec3 p, vEngine::BoundingBox originRect, vEngine::BoundingBox newRect );

protected:
    virtual ~Curve();

    vEngine::ref_ptr<vEngine::Vec3Array> _pathPts;
    vEngine::ref_ptr<AlgorithmCallback> _algorithmCallback;

    bool _updated;
};

}

#endif
