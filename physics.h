#include <Arduino.h>
#pragma once

namespace physics {

inline float fastCos( float x ) { return( 1-((x*x)/2.0f) ); }

template <class T = float>
class vec3
{
public:

    T x, y, z;

    vec3() { x = y = z = 0.0; }
    vec3(T x, T y, T z) { this->x = x, this->y = y, this->z = z; }

    operator String() const { return String(x) + "," + String(y) + "," + String(z); }

    vec3 &operator+=(const vec3 &v) {
        this->x += v.x;
        this->y += v.y;
        this->z += v.z;

        return(*this);    
    };
    const vec3 &operator+(const vec3 &v) const { return vec3(*this) += v; }

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

    vec3 &fnorm() {
        T normal = flen();

        this->x /= normal;
        this->y /= normal;
        this->z /= normal;

        return(*this);
    };

    inline T len() { return( sqrtf( x * x + y * y + z * z ) ); };
    // inline T flen() { return( fsqrtf( x * x + y * y + z * z ) ); }

    vec3 cross(const vec3 &v) {
        vec3<T> vNew = vec3<T>(this->y * v.z - this->z * v.y, this->z * v.x - this->x * v.z, this->x * v.y - this->y * v.x);
        return (vNew);
    }

    T dot(const vec3 &v) {
        return (this->x * v.x + this->y * v.y + this->z * v.z);
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

    inline Quat &operator*=(const Quat &other) {
        Quat<T> qNew;

        qNew.x =  this->x * other.w + this->y * other.z - this->z * other.y + this->w * other.x;
        qNew.y = -this->x * other.z + this->y * other.w + this->z * other.x + this->w * other.y;
        qNew.z =  this->x * other.y - this->y * other.x + this->z * other.w + this->w * other.z;
        qNew.w = -this->x * other.x - this->y * other.y - this->z * other.z + this->w * other.w;

        return (*this = qNew);
    }; // multiplication operators
    const inline Quat operator*(const Quat &other) const { return Quat(*this) *= other; }

    // Quat & operator*=(T s);
    // const Quat operator*(T s) const { return Quat(*this) *= s; } // scalar multiplication

    inline T norm() const { return (sqrtf(w * w + x * x + y * y + z * z)); } // norm of Quat
    inline T fnorm() const { return ( fsqrtf(w * w + x * x + y * y + z * z) ); } // fast norm of Quat
    
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

    Quat fnormalize()
    {
        T n = fnorm();
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

    Quat from_axis_angle_fast(T t, vec3<T> &eulerAngles) {
        T sn = t/2.0f;

        w = 1-((t*t)/2.0f);
        x = eulerAngles.x * sn;
        y = eulerAngles.y * sn;
        z = eulerAngles.z * sn;

        return(*this);
    }; // honestly just use this its basically just as accurate for quat updates

    Quat from_axis_angle(T t, const vec3<T> &eulerAngles) {
        T sn = sin(t/2.0);

        sizeof(t) == 4 ? w = cosf(t/2.0) : w = cos(t/2.0);
        x = eulerAngles.x * sn;
        y = eulerAngles.y * sn;
        z = eulerAngles.z * sn;

        return(*this);
    }; // from axis angles? lol

    Quat rotate(const Quat &q) const {
        Quat<T> qNew = (*this * q) * Quat<T>(this->w, -this->x, -this->y, -this->z);
        return qNew;
    };// input Quat ouput Quat

    Quat rotate(const vec3<T> &v) const {
        Quat<T> qNew = this.rotate(Quat<T>(1.0, v.x, v.y, v.z));
        return vec3<T>(qNew.x, qNew.y, qNew.z);
    } // input vec3 ouput quat

    vec3<T> rotateVec(const Quat &q) const {
        Quat<T> qNew = this.rotate(q);
        return vec3<T>(qNew.x, qNew.y, qNew.z);
    }; // input Quat ouput vec3

    vec3<T> rotateVec(const vec3<T> &v) const {
        Quat<T> qNew = this.rotate(Quat<T>(1.0, v.x, v.y, v.z));
        return vec3<T>(qNew.x, qNew.y, qNew.z);
    };// input vec3 ouput vec3

    const vec3<T> rotate_fast(const vec3<T> &v) {
        vec3<T> qVec = vec3<T>(this->x, this->y, this->z); // convert the quaternion into a vector of the real parts
        vec3<T> t = qVec.cross(v) * 2.0f; // 2 * cross product between the quaternion's XYZ components and the vector ( 2 * cross(q, v) )
        vec3<T> tCross = qVec.cross(t);
        vec3<T> ret = (t * this->w) + v + (tCross); 
        return ret;
    }; // s p e e d

    Quat rotation_between_vectors(const vec3<T> &v) {
        Quat<T> q = *this * Quat<T>(0, v.x, v.y, v.z);
        q.w = 1 - q.w;
        return(q.normalize());
    };   // rotation between two vectors

    const vec3<T> euler_angles() const {
        T r = atan2f( 2.0f * ( this->w * this->x + this->y * this->z ), 1.0f - 2.0f * ( this->x * this->x + this->y * this->y ) );
        T p = 2.0f * ( this->w * this->y - this->z * this->x );
        T y = atan2f( 2.0f * ( this->w * this->z + this->x * this->y ), 1.0f - 2.0f * ( this->y * this->y + this->z * this->z ) );
        vec3<T> ret = vec3<T>(r, p, y);
        return ret;
    };// convert Quat to euler angles
    
    const Quat<T> from_eulers(T roll, T pitch, T yaw) {

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

    const Quat<T> from_eulers_fast(T roll, T pitch, T yaw) {

        float cos_cutoff = 0.5; // cutoff for cosine angle approximation (0.5 gives < 0.15 degrees of error)
        float sin_cutoff = 0.5; // cutoff for sine angle approximation (0.5 gives < 0.15 degrees of error)

        T cr, cp, cy, sr, sp, sy;
        
        abs(roll)  < cos_cutoff ?  cr = fastCos( roll  / 2.0f ) : cr = cosf( roll   / 2.0f );
        abs(pitch) < cos_cutoff ?  cp = fastCos( pitch / 2.0f ) : cp = cosf( pitch  / 2.0f );
        abs(yaw)   < cos_cutoff ?  cy = fastCos( yaw   / 2.0f ) : cy = cosf( yaw    / 2.0f );

        abs(roll)  < sin_cutoff ? sr = roll  / 2.0f  : sr = sinf( roll  / 2.0f );
        abs(pitch) < sin_cutoff ? sp = pitch / 2.0f  : sp = sinf( pitch / 2.0f );
        abs(yaw)   < sin_cutoff ? sy = yaw   / 2.0f  : sy = sinf( yaw   / 2.0f );

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
