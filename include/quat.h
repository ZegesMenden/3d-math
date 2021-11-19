#ifndef QUAT_H
#define QUAT_H

#include <vector.h>
#include <arduino.h>

class Quaternion
{

    public:

        float w = 1.0, x, y, z;

        Quaternion(); // standard initializer
        Quaternion(float sw, float sx, float sy, float sz) { w = sw; x = sx; y = sy; z = sz; } // initializer with varibles
        
        Quaternion &operator=(const Quaternion &other) // assignment operator
        {
            w = other.w;
            x = other.x;
            y = other.y;
            z = other.z;
            return *this;
        }

        Quaternion & operator+=(const Quaternion &other) { Quaternion qNew(w + other.w, x + other.x, y + other.y, z + other.z); return qNew; } // addition operator
        const Quaternion operator+(const Quaternion &other) const { return Quaternion(*this) += other; } // addition operators

        Quaternion & operator-=(const Quaternion &other) { Quaternion qNew(w - other.w, x - other.x, y - other.y, z - other.z); return qNew; }
        const Quaternion operator-(const Quaternion &other) const { return Quaternion(*this) -= other; } // subtraction operators

        Quaternion & operator*=(const Quaternion &other); // multiplication operators
        const Quaternion operator*(const Quaternion &other) const { return Quaternion(*this) *= other; }

        Quaternion & operator*=(float s) { Quaternion qNew(w*s, x*s, y*s, z*s); return qNew; }
        const Quaternion operator*(float s) const { return Quaternion(*this) *= s; } // scalar multiplication
        
        float norm() const { return(sqrt(w*w + x*x + y*y + z*z)); } // norm of quaternion
        Quaternion conjugate() { return Quaternion(w, -x, -y, -z); } // conjugate of quaternion
        Quaternion normalize() { float n = norm(); return( Quaternion(w/n, x/n, y/n, z/n) ); } // normalize quaternion
        Quaternion fractional(float alpha);

        Quaternion from_euler(float pitch, float yaw, float roll); // convert euler angles to quaternion
        Quaternion from_euler(const vector3 &euler) { return from_euler(euler.x, euler.y, euler.z); } // convert euler angles to quaternion

        Quaternion from_axis_angle(float t, vector3 &eulerAngles); // from axis angles? lol

        vector3 rotate(const vector3 &v) const; // rotate vector by quaternion
        Quaternion rotate(const Quaternion &q) const; // rotate vector as quaternion by quaternion
        Quaternion rotation_between_vectors(const vector3 &v); // rotation between two vectors

        vector3 euler_angles() const; // convert quaternion to euler angles

};

#endif