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

#include <vEngine/Notify.h>
#include "Utilities.h"

using namespace osgModeling;

bool osgModeling::calcBoundAndCenter( vEngine::Vec3* ptr, unsigned int size, vEngine::Vec3* center, vEngine::BoundingBox* bound )
{
    if ( !ptr ) return false;

    vEngine::Vec3 first = *ptr;
    vEngine::Vec3 centerPoint(0.0f, 0.0f, 0.0f);
    vEngine::BoundingBox boundRect;
    unsigned int pNum = size;
    if ( pNum==1 )
    {
        centerPoint = first;
        boundRect.expandBy( first );
    }
    else
    {
        for ( unsigned int i=0; i<pNum-1; ++i )
        {
            centerPoint += *ptr;
            boundRect.expandBy( *ptr++ );
        }
        if ( first!=*ptr )
        {
            centerPoint += *ptr;
            centerPoint /= pNum;
            boundRect.expandBy( *ptr );
        }
        else
            centerPoint /= pNum-1;
    }

    if ( center ) *center = centerPoint;
    if ( bound ) *bound = boundRect;
    return true;
}

bool osgModeling::calcBoundAndCenter( vEngine::Vec3Array* pts, vEngine::Vec3* center, vEngine::BoundingBox* bound )
{
    if ( !pts ) return false;
    return calcBoundAndCenter( &(pts->front()), pts->size(), center, bound );
}

vEngine::Vec3 osgModeling::calcNormal( const vEngine::Vec3 l1, const vEngine::Vec3 l2, bool* ok )
{
    vEngine::Vec3 normal = l1 ^ l2;
    double len = normal.normalize();
    if ( ok ) *ok = len?true:false;
    return normal;
}

vEngine::Vec3 osgModeling::calcNormal( const vEngine::Vec3 p1, const vEngine::Vec3 p2, const vEngine::Vec3 p3, bool* ok )
{
    return osgModeling::calcNormal( p2-p1, p3-p1, ok );
}

vEngine::Vec3 osgModeling::calcProjection( const vEngine::Vec3 v, const vEngine::Vec3 target )
{
    double len2 = target.length2();
    if ( !len2 ) return vEngine::Vec3( 0.0f, 0.0f, 0.0f );

    double k = v * target;
    k /= target.length2();
    return target * k;
}

vEngine::Vec3 osgModeling::calcProjection( const vEngine::Vec3 v, const vEngine::Plane target )
{
    vEngine::Vec3 normal = target.getNormal();
    normal.normalize();

    double distance = v*normal + target[3];
    return v - normal * distance;
}

double osgModeling::calcAngle( const vEngine::Vec3 v1, const vEngine::Vec3 v2 )
{
    double base = v1.length() * v2.length();
    if ( !base ) return 0.0;

    return acos( (v1*v2) / base );
}

double osgModeling::calcAngle( const vEngine::Vec3 v, const vEngine::Plane p )
{
    return vEngine::PI_2 - calcAngle(v, p.getNormal());
}

double osgModeling::calcAngle( const vEngine::Plane p1, const vEngine::Plane p2 )
{
    return calcAngle( p1.getNormal(), p2.getNormal() );
}

vEngine::Vec3 osgModeling::calcIntersect( const vEngine::Vec3 p1, const vEngine::Vec3 v1, const vEngine::Vec3 p2, const vEngine::Vec3 v2, 
                                     bool checkCoplanar, bool* ok, bool* isCoin, double* pos )
{
    if ( ok ) *ok = false;
    if ( isCoin ) *isCoin = false;
    if ( pos ) *pos = 0.0f;
    if ( !v1.length2() || !v2.length2() ) return vEngine::Vec3( 0.0f, 0.0f, 0.0f );

    vEngine::Matrix3 m;
    vEngine::Vec3 p21 = p2 - p1, v12 = v1^v2;
    double s=0, t=0, base = v12.length2();
    bool coPlanar = true;

    // Check if 2 lines are co-planar.
    if ( checkCoplanar )
    {
        m.set( p21.x(), p21.y(), p21.z(),
               v1.x(),  v1.y(),  v1.z(),
               v2.x(),  v2.y(),  v2.z() );
        double det = determinant( m );
        if ( !vEngine::equivalent(det, (double)0.0f) )
            coPlanar = false;
    }

    if ( vEngine::equivalent(base, (double)0.0f) )
    {
        vEngine::Vec3 p21N = p21, v1N = v1;
        if ( p21N==vEngine::Vec3(0.0f,0.0f,0.0f) ) p21N = p2+v2 - p1;
        p21N.normalize();
        v1N.normalize();
        if ( !equivalent(p21N, v1N) )
            return vEngine::Vec3( 0.0f, 0.0f, 0.0f );
        else
        {
            if ( ok ) *ok = true;
            if ( isCoin ) *isCoin = true;

            // Check if the start/end point of a line is on the other.
            double* p = pos ? pos : &t;
            *p = (p1 - p2) * v2 / (v2.length2());
            if ( *p>=0.0f && *p<=1.0f ) return p1;
            *p = (p1+v1 - p2) * v2 / (v2.length2());
            if ( *p>=0.0f && *p<=1.0f ) return p1+v1;
            *p = (p2 - p1) * v1 / (v1.length2());
            if ( *p>=0.0f && *p<=1.0f ) return p2;
            *p = (p2+v2 - p1) * v1 / (v1.length2());
            if ( *p>=0.0f && *p<=1.0f ) return p2+v2;

            if ( ok ) *ok = false;
            if ( isCoin ) *isCoin = false;
            return vEngine::Vec3( 0.0f, 0.0f, 0.0f );
        }
    }

    // Fast line-line intersect test. Line: p(s) = p + s*v
    m.set( p21.x(), p21.y(), p21.z(),
           v2.x(),  v2.y(),  v2.z(),
           v12.x(), v12.y(), v12.z() );
    s = determinant( m ) / base;
    m.set( p21.x(), p21.y(), p21.z(),
           v1.x(),  v1.y(),  v1.z(),
           v12.x(), v12.y(), v12.z() );
    t = determinant( m ) / base;
    if ( ok && coPlanar && (s>=0.0f && s<=1.0f) && (t>=0.0f && t<=1.0f) )
        *ok = true;
    if ( pos ) *pos = s;
    return p1 + v1*s;
}

vEngine::Vec3 osgModeling::calcIntersect( const vEngine::Vec3 p, const vEngine::Vec3 v, const vEngine::Plane plane,
                                     bool* ok, bool* coplanar, double* pos )
{
    if ( ok ) *ok = false;
    if ( coplanar ) *coplanar = false;
    if ( pos ) *pos = 0.0f;

    double base = plane[0]*v.x() + plane[1]*v.y() + plane[2]*v.z();
    double t = plane.distance( p );
    if ( !base ) // 判断直线是否与平面平行
    {
        if ( ok && vEngine::equivalent(t,(double)0.0f) )
        {
            *ok = true;
            if ( coplanar ) *coplanar = true;
        }
        return p;
    }
    t = -t / base;
    if ( ok && t>=0.0f && t<=1.0f ) *ok = true;
    if ( pos ) *pos = t;
    return vEngine::Vec3( p.x()+t*v.x(), p.y()+t*v.y(), p.z()+t*v.z() );
}

vEngine::Plane osgModeling::calcPlane( const vEngine::Vec3 p1, const vEngine::Vec3 p2, const vEngine::Vec3 p3, bool* ok )
{
    vEngine::Vec3 normal = calcNormal( p1, p2, p3, ok );
    if ( ok && *ok==false ) return vEngine::Plane(0.0f, 0.0f, 0.0f, 0.0f);
    else return vEngine::Plane( normal, p1 );
}

vEngine::Matrix osgModeling::coordSystemMatrix( const vEngine::Vec3 orig,
                                               vEngine::Vec3 newX,
                                               vEngine::Vec3 newY,
                                               vEngine::Vec3 newZ )
{
    if ( !newX.length2() )
    {
        newY.normalize();
        newZ.normalize();
        newX = newY ^ newZ;
        newX.normalize();
    }
    else if ( !newY.length2() )
    {
        newX.normalize();
        newZ.normalize();
        newY = newZ ^ newX;
        newY.normalize();
    }
    else if ( !newZ.length2() )
    {
        newX.normalize();
        newY.normalize();
        newZ = newX ^ newY;
        newZ.normalize();
    }
    else
    {
        newX.normalize();
        newY.normalize();
        newZ.normalize();
    }

    return vEngine::Matrix(
        newX.x(), newX.y(), newX.z(), 0.0f,
        newY.x(), newY.y(), newY.z(), 0.0f,
        newZ.x(), newZ.y(), newZ.z(), 0.0f,
        orig.x(), orig.y(), orig.z(), 1.0f
        );
}

vEngine::Matrix osgModeling::rotateMatrix( vEngine::Vec3 axis, const double angle )
{
    double cosA = cos(angle);
    double sinA = sin(angle);
    axis.normalize();

    if ( axis==vEngine::Vec3(1.0f,0.0f,0.0f) )	// X
    {
        return vEngine::Matrix(
            1.0f,  0.0f,  0.0f,  0.0f,
            0.0f,  cosA,  -sinA, 0.0f,
            0.0f,  sinA,  cosA,  0.0f,
            0.0f,  0.0f,  0.0f,  1.0f
            );
    }
    else if ( axis==vEngine::Vec3(0.0f,1.0f,0.0f) )	// Y
    {
        return vEngine::Matrix(
            cosA,  0.0f,  sinA,  0.0f,
            0.0f,  1.0f,  0.0f,  0.0f,
            -sinA, 0.0f,  cosA,  0.0f,
            0.0f,  0.0f,  0.0f,  1.0f
            );
    }
    else if ( axis==vEngine::Vec3(0.0f,0.0f,1.0f) )	// Z
    {
        return vEngine::Matrix(
            cosA,  -sinA, 0.0f,  0.0f,
            sinA,  cosA,  0.0f,  0.0f,
            0.0f,  0.0f,  1.0f,  0.0f,
            0.0f,  0.0f,  0.0f,  1.0f
            );
    }

    double a1 = axis.x();
    double a2 = axis.y();
    double a3 = axis.z();

    // Rotate matrix of any vector, use the Rodriguez formula [Hecker 1997]
    // v' = v * cosA + axis*(v*axis) * (1-cosA) + (axis^vec) * sinA
    // So, T = I * cosA + (axis@axis) * (1-cosA) + M~ * sinA
    // @ means tensor product and M~ is a temp matrix of calculating cross product.
    return vEngine::Matrix(
        cosA+a1*a1*(1-cosA),    a2*a1*(1-cosA)-a3*sinA, a3*a1*(1-cosA)+a2*sinA, 0.0f,
        a1*a2*(1-cosA)+a3*sinA, cosA+a2*a2*(1-cosA),    a3*a2*(1-cosA)-a1*sinA, 0.0f,
        a1*a3*(1-cosA)-a2*sinA, a2*a3*(1-cosA)+a1*sinA, cosA+a3*a3*(1-cosA),    0.0f,
        0.0f,                   0.0f,                   0.0f,                   1.0f
        );
}

double osgModeling::checkOrientation( const vEngine::Vec3 v1, const vEngine::Vec3 v2, const vEngine::Vec3 ref )
{
    if ( !v1.length2() || !v2.length2() ) return 0.0f;

    if ( ref==vEngine::Vec3(0.0f,0.0f,1.0f) )
    {
        // Check the cross product of projections on XY plane.
        return v1.y()*v2.x() - v1.x()*v2.y();
    }
    else if ( ref==vEngine::Vec3(1.0f,0.0f,0.0f) )
    {
        // Check the cross product of projections on YZ plane.
        return v1.z()*v2.y() - v1.y()*v2.z();
    }
    else if ( ref==vEngine::Vec3(0.0f,1.0f,0.0f) )
    {
        // Check the cross product of projections on XZ plane.
        return v1.x()*v2.z() - v1.z()*v2.x();
    }
    else
    {
        vEngine::Plane refPlane( ref, vEngine::Vec3(0.0f,0.0f,0.0f) );
        vEngine::Vec3 v1p = calcProjection( v1, refPlane );
        vEngine::Vec3 v2p = calcProjection( v2, refPlane );

        vEngine::Vec3 v = v1p^v2p;
        if ( equivalent(v) ) return 0.0f;
        v.normalize();
        if ( (v+ref).length2()<ref.length2() ) return -1.0f;
        else return 1.0f;
    }
}

double osgModeling::determinant( vEngine::Matrix2 m )
{
    return m[0]*m[3] - m[1]*m[2];
}

double osgModeling::determinant( vEngine::Matrix3 m )
{
    return m[0]*m[4]*m[8] + m[1]*m[5]*m[6] + m[2]*m[3]*m[7]
        - m[2]*m[4]*m[6] - m[1]*m[3]*m[8] - m[0]*m[5]*m[7];
}

double osgModeling::factorial( const int n, bool warnLargeValue )
{
    static double constNum[7] = { 1.0f, 1.0f, 2.0f, 6.0f, 24.0f, 120.0f, 720.0f };
    if ( n<7 )
    {
        if ( n<0 ) return -1.0f;
        return constNum[n];
    }
    else if ( n>32 && warnLargeValue )
    {
        vEngine::notify(vEngine::WARN) << "osgModeling: " << n << "! is rarely used when calculating factorial. "
            << "Is it correct?" << std::endl;
        return -1.0f;
    }

    int i=7;
    double result = constNum[6];
    while ( i<=n )
        result *= i++;
    return result;
}
