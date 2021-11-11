#ifndef VECTORMATH_h
#include VECTORMATH_h

#include <Arduino.h>

class vector3
{
    public:
    float x, y, z;

    vector3() { x = y = z = 0.0; }
    vector3(float nx, float ny, float nz) { x = nx, y = ny, z = nz; }

    vector3 & operator+=( const vector3& v);
    const vector3 & operator+( const vector3& v) const { return vector3(*this) += v; }

    vector3 & operator-=( const vector3& v);
    const vector3 & operator-( const vector3& v) const { return vector3(*this) -= v; }

    vector3 & operator*=( const vector3& v);
    const vector3 & operator*( const vector3& v) const { return vector3(*this) *= v; }

    vector3 & operator*=( float scale );
    const vector3 & operator*( float scale ) const { return vector3(*this) *= scale; }

    vector3 & operator/=( const vector3& v);
    const vector3 & operator/( const vector3& v) const { return vector3(*this) /= v; }

    vector3 & operator/=( float scale );
    const vector3 & operator/( float scale ) const { return vector3(*this) /= scale; }

    vector3 & norm();
    float len();
};

#endif