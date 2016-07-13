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

#include <iostream>
#include <algorithm>
#include <vEngine/TriangleFunctor.h>
#include "Utilities.h"
#include "Model.h"
#include "NormalVisitor.h"

using namespace osgModeling;

struct CalcNormalFunctor
{
    typedef std::multiset<const vEngine::Vec3*, LessPtr> CoordinateSet;
    unsigned int _coordSize;
    CoordinateSet _coordSet;
    vEngine::Vec3* _coordBase;

    // Normal calculating variables & functions.
    bool _flip;
    int _method;
    double _threshold;
    vEngine::Vec3* _normalBase;
    std::vector<vEngine::Vec3> _lastNormalRecorder;

    void setNormalParameters( vEngine::Vec3* nb, bool flip, int method, double t )
    {
        _normalBase = nb;
        _flip = flip;
        _method = method;
        _threshold = t;
    }

    inline void incNormal( const vEngine::Vec3& vec, const vEngine::Vec3& normal, double weight )
    {
        std::pair<CoordinateSet::iterator, CoordinateSet::iterator> p =
            _coordSet.equal_range( &vec );

        for ( CoordinateSet::iterator itr=p.first; itr!=p.second; ++itr )
        {
            int pos = *itr - _coordBase;
            double t = normal * _lastNormalRecorder[pos];
            if ( _threshold<1.0f )
            {
                if ( !equivalent(_lastNormalRecorder[pos], vEngine::Vec3(0.0f,0.0f,0.0f)) 
                    && t<_threshold && t>-_threshold )
                    continue;
            }

            vEngine::Vec3* nptr = _normalBase + pos;
            *nptr += normal * weight;
            _lastNormalRecorder[pos] = normal;
        }
    }

    // General functions.
    CalcNormalFunctor():
        _coordSize(0), _coordBase(0)
    {}

    void setVerticsPtr( vEngine::Vec3* cb, unsigned int cs )
    {
        _coordSize = cs;
        _coordBase = cb;

        vEngine::Vec3* vptr = cb;
        for ( unsigned int i=0; i<cs; ++i )
        {
            _coordSet.insert( vptr++ );
            _lastNormalRecorder.push_back( vEngine::Vec3(0.0f,0.0f,0.0f) );
        }
    }

    inline void operator() ( const vEngine::Vec3& v1, const vEngine::Vec3& v2, const vEngine::Vec3& v3, bool treatVertexDataAsTemporary )
    {
        if ( treatVertexDataAsTemporary || v1==v2 || v1==v3 || v2==v3 )
            return;

        double w[3]= { 1.0f, 1.0f, 1.0f };
        switch ( _method )
        {
        case NormalVisitor::MWA:
            w[0] = asin( ((v2-v1)^(v3-v1)).length()/((v2-v1).length()*(v3-v1).length()) );
            w[1] = asin( ((v3-v2)^(v1-v2)).length()/((v3-v2).length()*(v1-v2).length()) );
            w[2] = asin( ((v1-v3)^(v2-v3)).length()/((v1-v3).length()*(v2-v3).length()) );
            break;
        case NormalVisitor::MWSELR:
            w[0] = ((v2-v1)^(v3-v1)).length()/((v2-v1).length2()*(v3-v1).length2());
            w[1] = ((v3-v2)^(v1-v2)).length()/((v3-v2).length2()*(v1-v2).length2());
            w[2] = ((v1-v3)^(v2-v3)).length()/((v1-v3).length2()*(v2-v3).length2());
            break;
        case NormalVisitor::MWAAT:
            w[0] = ((v2-v1)^(v3-v1)).length();
            w[1] = ((v3-v2)^(v1-v2)).length();
            w[2] = ((v1-v3)^(v2-v3)).length();
            break;
        case NormalVisitor::MWELR:
            w[0] = 1/((v2-v1).length()*(v3-v1).length());
            w[1] = 1/((v3-v2).length()*(v1-v2).length());
            w[2] = 1/((v1-v3).length()*(v2-v3).length());
            break;
        case NormalVisitor::MWSRELR:
            w[0] = 1/sqrt((v2-v1).length()*(v3-v1).length());
            w[1] = 1/sqrt((v3-v2).length()*(v1-v2).length());
            w[2] = 1/sqrt((v1-v3).length()*(v2-v3).length());
            break;
        default:
            break;
        }
        
        vEngine::Vec3 normal = (v2-v1)^(v3-v1) * (_flip?-1.0f:1.0f);
        incNormal( v1, normal, w[0] );
        incNormal( v2, normal, w[1] );
        incNormal( v3, normal, w[2] );
    }
};

NormalVisitor::NormalVisitor( int method, bool flip )
{
    _threshold = 1e-6f;
    _method = method;
    _flip = flip;
    setTraversalMode( vEngine::NodeVisitor::TRAVERSE_ALL_CHILDREN );
}

NormalVisitor::~NormalVisitor()
{
}

bool NormalVisitor::checkPrimitives( vEngine::Geometry& geom )
{
    vEngine::Geometry::PrimitiveSetList& primitives = geom.getPrimitiveSetList();
    vEngine::Geometry::PrimitiveSetList::iterator itr;
    unsigned int numSurfacePrimitives=0;
    for ( itr=primitives.begin(); itr!=primitives.end(); ++itr )
    {
        switch ( (*itr)->getMode() )
        {
        case (vEngine::PrimitiveSet::TRIANGLES):
        case (vEngine::PrimitiveSet::TRIANGLE_STRIP):
        case (vEngine::PrimitiveSet::TRIANGLE_FAN):
        case (vEngine::PrimitiveSet::QUADS):
        case (vEngine::PrimitiveSet::QUAD_STRIP):
        case (vEngine::PrimitiveSet::POLYGON):
            ++numSurfacePrimitives;
            break;
        default:
            break;
        }
    }

    if ( !numSurfacePrimitives ) return false;
    return true;
}

void NormalVisitor::buildNormal( vEngine::Geometry& geom, bool flip, int method, double threshold )
{
    if ( !checkPrimitives(geom) ) return;

    vEngine::Vec3Array *coords = dynamic_cast<vEngine::Vec3Array*>( geom.getVertexArray() );
    if ( !coords || !coords->size() ) return;

    vEngine::Vec3Array::iterator nitr;
    vEngine::Vec3Array *normals = new vEngine::Vec3Array( coords->size() );
    for ( nitr=normals->begin(); nitr!=normals->end(); ++nitr )
    {
        nitr->set( 0.0f, 0.0f, 0.0f );
    }

    vEngine::TriangleFunctor<CalcNormalFunctor> ctf;
    ctf.setVerticsPtr( &(coords->front()), coords->size() );
    ctf.setNormalParameters( &(normals->front()), flip, method, threshold );
    geom.accept( ctf );

    for ( nitr=normals->begin(); nitr!=normals->end(); ++nitr )
    {
        nitr->normalize();
    }

    geom.setNormalArray( normals );
    //geom.setNormalIndices( geom.getVertexIndices() );
    geom.setNormalBinding( vEngine::Geometry::BIND_PER_VERTEX );
}

void NormalVisitor::apply(vEngine::Geode& geode)
{
    for(unsigned int i = 0; i < geode.getNumDrawables(); i++ )
    {
      vEngine::Geometry* geom = dynamic_cast<vEngine::Geometry*>( geode.getDrawable(i) );
      if ( geom ) buildNormal( *geom, _flip, _method, _threshold );
    }
}

