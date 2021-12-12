#ifndef PHYSICS_H
#define PHYSICS_H

#include <Arduino.h>
// #include <physics.h>

#pragma once

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

        vector3 from_quaternion(const Quaternion &q);

        // Quaternion to_quaternion() { return Quaternion(0.0, x, y, z); }

};

class Quaternion
{

    public:

        float w, x, y, z;

        Quaternion() { w = 1.0; x = 0.0; y = 0.0; z = 0.0; }
        Quaternion(float sw, float sx, float sy, float sz) { w = sw; x = sx; y = sy; z = sz; } // initializer with varibles
        
        Quaternion &operator=(const Quaternion &other) // assignment operator
        {
            w = other.w;
            x = other.x;
            y = other.y;
            z = other.z;
            return *this;
        }

        Quaternion & operator+=(const Quaternion &other);
        const Quaternion operator+(const Quaternion &other) const { return Quaternion(*this) += other; } // addition operators

        Quaternion & operator-=(const Quaternion &other);
        const Quaternion operator-(const Quaternion &other) const { return Quaternion(*this) -= other; } // subtraction operators

        Quaternion & operator*=(const Quaternion &other); // multiplication operators
        const Quaternion operator*(const Quaternion &other) const { return Quaternion(*this) *= other; }

        // Quaternion & operator*=(float s);
        // const Quaternion operator*(float s) const { return Quaternion(*this) *= s; } // scalar multiplication
        
        float norm() const { return(sqrt(w*w + x*x + y*y + z*z)); } // norm of quaternion
        Quaternion conjugate() { return Quaternion(w, -x, -y, -z); } // conjugate of quaternion
        Quaternion normalize() { float n = norm(); return( Quaternion(w/n, x/n, y/n, z/n) ); } // normalize quaternion
        Quaternion fractional(float alpha);

        Quaternion from_euler(float pitch, float yaw, float roll); // convert euler angles to quaternion
        // Quaternion from_euler(const vector3 &euler) const { return from_euler(euler.x, euler.y, euler.z); } // convert euler angles to quaternion

        Quaternion from_axis_angle(float t, vector3 &eulerAngles); // from axis angles? lol

        vector3 rotate(const vector3 &v) const; // rotate vector by quaternion
        Quaternion rotate_quaternion(const Quaternion &q) const; // rotate vector as quaternion by quaternion
        Quaternion rotation_between_vectors(const vector3 &v); // rotation between two vectors

        const Quaternion euler_angles(); // convert quaternion to euler angles
        const Quaternion from_vector3(const vector3 &v); // converts a vector to a quaternion

};

// Quaternion vector_to_quat(vector3 v) { return Quaternion(0.0, v.x, v.y, v.z); }
// vector3 quat_to_vector(Quaternion q) { return vector3(q.x, q.y, q.z); }

