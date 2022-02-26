#include <Arduino.h>

#pragma once

namespace physics {
    
template <class T = float>
class vec3
{
public:

    T x, y, z;

    vec3() { x = y = z = 0.0; }
    vec3(T x, T y, T z) { this->x = x, this->y = y, this->z = z; }

    const vec3 &operator+(const vec3 &v) const {
        vec3<T> ret;
        ret.x = this->x + v.x;
        ret.y = this->y + v.y;
        ret.z = this->z + v.z;

        return ret;
    }

    const vec3 &operator+=(const vec3 &v) { return (*this = *this + v); }

    // vec3 &operator+=(const vec3 &v) {
    //     vec3<T> vNew;
    //     vNew.x = this->x + v.x;
    //     vNew.y = this->y + v.y;
    //     vNew.z = this->z + v.z;

    //     return(*this = vNew);  
    // };
    // const vec3 &operator+(const vec3 &v) const { return vec3(*this) += v; }

    vec3 &operator-=(const vec3 &v) {
        this->x -= v.x;
        this->y -= v.y;
        this->z -= v.z;

        return(*this);    
    };
    const vec3 &operator-(const vec3 &v) const { return vec3(*this) -= v; }

    vec3 &operator*=(const vec3 &v) {
        this->x *= v.x;
        this->y *= v.y;
        this->z *= v.z;

        return(*this);
    };
    const vec3 &operator*(const vec3 &v) const { return vec3(*this) *= v; }

    vec3 &operator*=(T scale) {
        this->x *= scale;
        this->y *= scale;
        this->z *= scale;

        return(*this);
    };
    const vec3 &operator*(T scale) const { return vec3(*this) *= scale; }

    vec3 &operator/=(const vec3 &v) {
        this->x /= v.x;
        this->y /= v.y;
        this->z /= v.z;

        return(*this);
    };
    const vec3 &operator/(const vec3 &v) const { return vec3(*this) /= v; }

    vec3 &operator/=(T scale) {
        this->x /= scale;
        this->y /= scale;
        this->z /= scale;

        return(*this);
    };
    const vec3 &operator/(T scale) const { return vec3(*this) /= scale; }

    vec3 &norm() {
        T normal = len();

        this->x /= normal;
        this->y /= normal;
        this->z /= normal;

        return(*this);
    };

    T len() { return( sizeof(T) == sizeof(float) ? sqrtf( x * x + y * y + z * z) : sqrt( x * x + y * y + z * z) ); };

    vec3 cross(const vec3 &v) {
        vec3<T> vNew = vec3<T>(this->y * v.z - this->z * v.y, this->z * v.x - this->x * v.z, this->x * v.y - this->y * v.x);
        return (vNew);
    }

};

template <class T = float>
class Quat
{

public:
    T w, x, y, z;

    Quat()
    {
        w = 1.0;
        x = 0.0;
        y = 0.0;
        z = 0.0;
    }

    Quat(float sw, float sx, float sy, float sz)
    {
        w = sw;
        x = sx;
        y = sy;
        z = sz;
    } // initializer with varibles

    Quat &operator=(const Quat &other) // assignment operator
    {
        w = other.w;
        x = other.x;
        y = other.y;
        z = other.z;
        return *this;
    }

    Quat &operator+=(const Quat &other) {
   
        w += other.w;
        x += other.x;
        y += other.y;
        z += other.z;
        
        return (*this);
    };
    const Quat operator+(const Quat &other) const { return Quat(*this) += other; } // addition operators

    Quat &operator-=(const Quat &other) {
        w -= other.w;
        x -= other.x;
        y -= other.y;
        z -= other.z;
        
        return (*this);
    };
    const Quat operator-(const Quat &other) const { return Quat(*this) -= other; } // subtraction operators

    Quat &operator*=(const Quat &other) {
        Quat<T> qNew;

        qNew.x =  this->x * other.w + this->y * other.z - this->z * other.y + this->w * other.x;
        qNew.y = -this->x * other.z + this->y * other.w + this->z * other.x + this->w * other.y;
        qNew.z =  this->x * other.y - this->y * other.x + this->z * other.w + this->w * other.z;
        qNew.w = -this->x * other.x - this->y * other.y - this->z * other.z + this->w * other.w;

        return (*this = qNew);
    }; // multiplication operators
    const Quat operator*(const Quat &other) const { return Quat(*this) *= other; }

    // Quat & operator*=(T s);
    // const Quat operator*(T s) const { return Quat(*this) *= s; } // scalar multiplication

    T norm() const { return (sqrt(w * w + x * x + y * y + z * z)); } // norm of Quat

    Quat conjugate() { 
        this-> x = -x;
        this-> y = -y;
        this-> z = -z;
        return (*this);
    }         // conjugate of Quat

    Quat normalize()
    {
        T n = norm();
        return (Quat(w / n, x / n, y / n, z / n));
    } // normalize Quat

    Quat fractional(T alpha) {
        this->w = 1-alpha + alpha*w;
        this->x *= alpha;
        this->y *= alpha;
        this->z *= alpha;

        return normalize();
    };

    // Quat from_euler(T pitch, T yaw, T roll); // convert euler angles to Quat
    // Quat from_euler(const vec3 &euler) const { return from_euler(euler.x, euler.y, euler.z); } // convert euler angles to Quat

    Quat from_axis_angle(T t, vec3<T> &eulerAngles) {
        T sn = sin(t/2.0);

        sizeof(t) == 4 ? w = cosf(t/2.0) : w = cos(t/2.0);
        x *= sn;
        y *= sn;
        z *= sn;

        return(*this);
    }; // from axis angles? lol

    vec3<T> rotate(const vec3<T> &v) const {
        Quat<T> q(1.0f, v.x, v.y, v.z);
        Quat<T> qNew = (*this * q) * Quat<T>(this->w, -this->x, -this->y, -this->z);

        return vec3<T>(qNew.x, qNew.y, qNew.z);
    };// rotate vector by Quat

    Quat rotate_Quat(const Quat &q) const {
        Quat<T> qNew = (*this * q) * Quaternion(this->w, -this->x, -this->y, -this->z);
        return qNew;
    }; // rotate vector as Quat by Quat

    Quat rotation_between_vectors(const vec3<T> &v) {
        Quat<T> q = *this * Quat<T>(0, v.x, v.y, v.z);
        q.w = 1 - q.w;
        return(q.normalize());
    };   // rotation between two vectors

    const vec3<T> rotate_fast(const vec3<T> &v) {
        vec3<T> qVec = vec3<T>(this->x, this->y, this->z); // convert the quaternion into a vector of the real parts
        vec3<T> t = qVec.cross(v) * 2.0f; // 2 * cross product between the quaternion's XYZ components and the vector ( 2 * cross(q, v) )
        vec3<T> tCross = qVec.cross(t);
        vec3<T> ret = (t * this->w) + v + (tCross); 
        return ret;
    }; // s p e e d

    const Quat<T> euler_angles() const {
        T r = atan2f( 2.0 * ( this->w * this->x + this->y * this->z ), 1.0 - 2.0 * ( this->x * this->x + this->y * this->y ) );
        T p = 2.0 * ( this->w * this->y - this->z * this->x );
        T y = atan2f( 2.0 * ( this->w * this->z + this->x * this->y ), 1.0 - 2.0 * ( this->y * this->y + this->z * this->z ) );
        
        return (*this);
    };// convert Quat to euler angles
    
    const Quat<T> from_eulers(T roll, T pitch, T yaw) {
        
        // T cr = sizeof(T) == 4 ? cosf( roll  / 2 ) : cos( roll / 2 );
        // T cp = sizeof(T) == 4 ? cosf( pitch / 2 ) : cos( pitch / 2 );
        // T cy = sizeof(T) == 4 ? cosf( yaw   / 2 ) : cos( yaw / 2 );

        // T sr = sizeof(T) == 4 ? sinf( roll  / 2 ) : sin( roll / 2 );
        // T sp = sizeof(T) == 4 ? sinf( pitch / 2 ) : sin( pitch / 2 );
        // T sy = sizeof(T) == 4 ? sinf( yaw   / 2 ) : sin( yaw / 2 );

        T cr = cosf( roll  / 2.0f );
        T cp = cosf( pitch / 2.0f );
        T cy = cosf( yaw   / 2.0f );
        T sr = sinf( roll  / 2.0f );
        T sp = sinf( pitch / 2.0f );
        T sy = sinf( yaw   / 2.0f );

        this->w = cr * cp * cy + sr * sp * sy;
        this->x = sr * cp * cy - cr * sp * sy;
        this->y = cr * sp * cy + sr * cp * sy;
        this->z = cr * cp * sy - sr * sp * cy;

        return (*this);
    
    }

};

using Quaternion =  Quat<float>;
using Quaternionf = Quat<float>;
using Quaterniond = Quat<double>;

using vector3 =  vec3<float>;
using vector3f = vec3<float>;
using vector3d = vec3<double>;

template <class T = float>
vec3<T> quat2vec(const Quat<T> &q) { return vec3<T>(q.x, q.y, q.z); };

template <class T = float>
Quat<T> vec2quat(const vec3<T> &v) { return Quat<T>(1.0, v.x, v.y, v.z); };

}
