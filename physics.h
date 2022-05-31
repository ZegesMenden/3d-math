#pragma once

// include cmath if this is running on a desktop
#ifndef ARDUINO
#include <math.h>
#endif

namespace physics {

    template <class T = float>
    class vec3
    {
    public:

        T x, y, z;

        /**
         * @brief Construct a new vec3 object
         */
        vec3() { x = y = z = 0.0; }

        /**
         * @brief Construct a new vec3 object
         *
         * @param x x component
         * @param y y component
         * @param z z component
         */
        vec3(T x, T y, T z) { this->x = x, this->y = y, this->z = z; }

        /**
         * @brief Adds the right handed vector to the left handed vector
         * @return sum of the two vectors
         */
        vec3& operator+=(const vec3& v) {
            this->x += v.x;
            this->y += v.y;
            this->z += v.z;

            return(*this);
        };

        /**
         * @brief Adds two vectors
         * @return sum of the two vectors
         */
        const vec3& operator+(const vec3& v) const { return vec3(*this) += v; }

        /**
         * @brief Subtracts the right handed vector from the left handed vector
         * @return difference of the two vectors
         */
        vec3& operator-=(const vec3& v) {
            this->x -= v.x;
            this->y -= v.y;
            this->z -= v.z;

            return(*this);
        };

        /**
         * @brief Adds the right handed vector to the left handed vector
         * @return sum of the two vectors
         */
        const vec3& operator-(const vec3& v) const { return vec3(*this) -= v; }

        /**
         * @brief Multiplies the left handed vector by the right handed vector
         * @return product of the two vectors
         */
        vec3& operator*=(const vec3& v) {
            this->x *= v.x;
            this->y *= v.y;
            this->z *= v.z;

            return(*this);
        };

        /**
         * @brief Multiplies two vectors
         * @return product of the two vectors
         */
        const vec3& operator*(const vec3& v) const { return vec3(*this) *= v; }

        /**
         *  @brief Multiplies the left handed vector by a scalar
         * @return product of the vector and scalar
         */
        vec3& operator*=(T scale) {
            this->x *= scale;
            this->y *= scale;
            this->z *= scale;

            return(*this);
        };
        /**
         * @brief Multiplies a vector by a scalar
         * @return product of the vector and scalar
         */
        const vec3& operator*(T scale) const { return vec3(*this) *= scale; }

        /**
         * @brief Divides the left handed vector by the right handed vector
         * @return quotient of the two vectors
         */
        vec3& operator/=(const vec3& v) {
            this->x /= v.x;
            this->y /= v.y;
            this->z /= v.z;

            return(*this);
        };

        /**
         * @brief Divides two vectors
         * @return quotient of the two vectors
         */
        const vec3& operator/(const vec3& v) const { return vec3(*this) /= v; }

        /**
         * @brief Divides the left handed vector by a scalar
         * @return quotient of the vector and scalar
         */
        vec3& operator/=(T scale) {
            this->x /= scale;
            this->y /= scale;
            this->z /= scale;

            return(*this);
        };

        /**
         * @brief Divides a vector by a scalar
         * @return quotient of the vector and scalar
         */
        const vec3& operator/(T scale) const { return vec3(*this) /= scale; }

        /**
         * @brief Normalizes the vector
         * @return normalized vector
         */
        vec3& norm() {
            T normal = len();

            this->x /= normal;
            this->y /= normal;
            this->z /= normal;

            return(*this);
        };

        /**
         * @brief Returns the length of the vector
         * @return length of the vector
         */
        inline T len() { return(sqrtf(x * x + y * y + z * z)); };

        /**
         * @brief Returns the cross product of the left and right hand vectors
         * @return cross product of the two vectors
         */
        vec3 cross(const vec3& v) {
            vec3<T> vNew = vec3<T>(this->y * v.z - this->z * v.y, this->z * v.x - this->x * v.z, this->x * v.y - this->y * v.x);
            return (vNew);
        }

        /**
         * @brief Returns the dot product of the left and right hand vectors
         * @return dot product of the two vectors
         */
        T dot(const vec3& v) {
            return (this->x * v.x + this->y * v.y + this->z * v.z);
        }

    };

    template <class T = float>
    class Quat
    {

    public:
        T w, x, y, z;

        /**
         * @brief Constructs a new Quat object
         */
        Quat()
        {
            w = 1.0;
            x = 0.0;
            y = 0.0;
            z = 0.0;
        }

        /**
         * @brief Constructs a new Quat object
         *
         * @param w w component
         * @param x x component
         * @param y y component
         * @param z z component
         */
        Quat(float sw, float sx, float sy, float sz)
        {
            w = sw;
            x = sx;
            y = sy;
            z = sz;
        } // initializer with varibles

        /**
         * @brief Sets the quaternion equal to another quaternion
         *
         * @param other quat to copy from
         */
        Quat& operator=(const Quat& other) // assignment operator
        {
            w = other.w;
            x = other.x;
            y = other.y;
            z = other.z;
            return *this;
        }

        /**
         * @brief Adds the right hand quaternion to the left hand quaternion
         *
         * @param other quat to add
         * @return quat sum
         */
        Quat& operator+=(const Quat& other) {

            w += other.w;
            x += other.x;
            y += other.y;
            z += other.z;

            return (*this);
        };

        /**
         * @brief Adds two quaternions
         *
         * @param other quat to add
         * @return quat sum
         */
        const Quat operator+(const Quat& other) const { return Quat(*this) += other; } // addition operators

        /**
         * @brief Subtracts the right hand quaternion from the left hand quaternion
         *
         * @param other quat to subtract
         * @return quat difference
         */
        Quat& operator-=(const Quat& other) {
            w -= other.w;
            x -= other.x;
            y -= other.y;
            z -= other.z;

            return (*this);
        };

        /**
         * @brief Subtracts two quaternions
         *
         * @param other quat to subtract
         * @return quat difference
         */
        const Quat operator-(const Quat& other) const { return Quat(*this) -= other; } // subtraction operators

        /**
         * @brief Computes the hamiltonian product of the left hand quaternion by the right hand quaternion
         * @param other quat to multiply by
         * @return hamiltonian product
         */
        inline Quat& operator*=(const Quat& other) {
            Quat<T> qNew;

            qNew.x = this->x * other.w + this->y * other.z - this->z * other.y + this->w * other.x;
            qNew.y = -this->x * other.z + this->y * other.w + this->z * other.x + this->w * other.y;
            qNew.z = this->x * other.y - this->y * other.x + this->z * other.w + this->w * other.z;
            qNew.w = -this->x * other.x - this->y * other.y - this->z * other.z + this->w * other.w;

            return (*this = qNew);
        }; // multiplication operators

        /**
         * @brief Computes the hamiltonian product of the left hand quaternion by the right hand quaternion
         * @param other quat to multiply by
         * @return hamiltonian product
         */
        const inline Quat operator*(const Quat& other) const { return Quat(*this) *= other; }

        // Quat & operator*=(T s);
        // const Quat operator*(T s) const { return Quat(*this) *= s; } // scalar multiplication

        /**
         * @brief Calculates the length of the quaternion
         * @return length of the quaternion
         */
        inline T norm() const { return (sqrtf(w * w + x * x + y * y + z * z)); } // norm of Quat
        
        /**
         * @brief Computes the conjugate of the quaternion
         * @return conjugate of the quaternion
         */
        Quat conjugate() {
            this->x = -x;
            this->y = -y;
            this->z = -z;
            return (*this);
        }         // conjugate of Quat

        /**
         * @brief Normalizes the quaternion
         * @return normalized quaternion
         */
        Quat normalize()
        {
            T n = norm();
            return (Quat(w / n, x / n, y / n, z / n));
        } // normalize Quat

        /**
         * @brief Computes the fractional of the quaternon
         * @return fractional of the quaternion
         */
        Quat fractional(T alpha) {

            this->w = 1 - alpha + alpha * w;
            this->x *= alpha;
            this->y *= alpha;
            this->z *= alpha;

            return this->normalize();
        };

        /**
         * @brief Computes the quaternion from a set of axis angles and magnitude
         * @param t magnitude
         * @param angles normalized axis angles
         * @return quaternion
         */
        Quat from_axis_angle(T t, const vec3<T>& angles) {
            T sn = sin(t / 2.0);

            w = cosf(t / 2.0);
            x = angles.x * sn;
            y = angles.y * sn;
            z = angles.z * sn;

            return(*this);
        }; // from axis angles? lol

        /**
         * @brief rotates another quaternion by this quaternion
         * @param q quaternion to rotate
         * @return rotated quaternion as a quaternion
         */
        Quat rotate(const Quat& q) const {
            Quat<T> qNew = (*this * q) * Quat<T>(this->w, -this->x, -this->y, -this->z);
            return qNew;
        };// input Quat ouput Quat

        /**
         * @brief rotates a vector by this quaternion
         * @param v vector to rotate
         * @return rotated vector as a quaternion
         */
        Quat rotate(const vec3<T>& v) const {
            Quat<T> qNew = this.rotate(Quat<T>(1.0, v.x, v.y, v.z));
            return Quat<T>(0.0f, qNew.x, qNew.y, qNew.z);
        } // input vec3 ouput quat

        /**
         * @brief rotates a vector by this quaternion
         * @param v vector to rotate
         * @return rotated vector
         */
        vec3<T> rotateVec(const Quat& q) const {
            Quat<T> qNew = this.rotate(q);
            return vec3<T>(qNew.x, qNew.y, qNew.z);
        }; // input Quat ouput vec3

        /**
         * @brief rotates a vector by this quaternion
         * @param v vector to rotate
         * @return rotated vector
         */
        vec3<T> rotateVec(const vec3<T>& v) const {
            Quat<T> qNew = this.rotate(Quat<T>(1.0, v.x, v.y, v.z));
            return vec3<T>(qNew.x, qNew.y, qNew.z);
        };// input vec3 ouput vec3

        /**
         * @brief rotates a vector by this quaternion
         * @param v vector to rotate
         * @return rotated vector
         */
        const vec3<T> rotate_fast(const vec3<T>& v) {
            vec3<T> qVec = vec3<T>(this->x, this->y, this->z); // convert the quaternion into a vector of the real parts
            vec3<T> t = qVec.cross(v) * 2.0f; // 2 * cross product between the quaternion's XYZ components and the vector ( 2 * cross(q, v) )
            vec3<T> tCross = qVec.cross(t);
            vec3<T> ret = (t * this->w) + v + (tCross);
            return ret;
        }; // s p e e d

        /**
         * @brief Calculates the rotation between this quaternion and a vector
         * @param v vector to compare rotation to
         * @return the rotation between the two vectors
         */
        Quat rotation_between_vectors(const vec3<T>& v) {
            Quat<T> q = *this * Quat<T>(0, v.x, v.y, v.z);
            q.w = 1 - q.w;
            return(q.normalize());
        };   // rotation between two vectors

        /**
         * @brief Converts this quaternion to a set of euler angles
         * @return vec3 containing the euler angles
         */
        const vec3<T> euler_angles() const {
            T r = atan2f(2.0f * (this->w * this->x + this->y * this->z), 1.0f - 2.0f * (this->x * this->x + this->y * this->y));
            T p = 2.0f * (this->w * this->y - this->z * this->x);
            T y = atan2f(2.0f * (this->w * this->z + this->x * this->y), 1.0f - 2.0f * (this->y * this->y + this->z * this->z));
            vec3<T> ret = vec3<T>(r, p, y);
            return ret;
        };// convert Quat to euler angles

        /**
         * @brief Converts a set of euler angles to a quaternion
         * @param roll angle around the roll axis
         * @param pitch angle around the pitch axis
         * @param yaw angle around the yaw axis
         * @return the calculatedquaternion
         */
        const Quat<T> from_eulers(T roll, T pitch, T yaw) {

            T cr = cosf(roll / 2.0f);
            T cp = cosf(pitch / 2.0f);
            T cy = cosf(yaw / 2.0f);
            T sr = sinf(roll / 2.0f);
            T sp = sinf(pitch / 2.0f);
            T sy = sinf(yaw / 2.0f);

            this->w = cr * cp * cy + sr * sp * sy;
            this->x = sr * cp * cy - cr * sp * sy;
            this->y = cr * sp * cy + sr * cp * sy;
            this->z = cr * cp * sy - sr * sp * cy;

            return (*this);

        }

        /**
         * @brief Converts a set of euler angles to a quaternion
         * @param roll angle around the roll axis
         * @param pitch angle around the pitch axis
         * @param yaw angle around the yaw axis
         * @return the calculatedquaternion
         */
        const Quat<T> from_eulers(vec3<T>& eulers) { return this->from_eulers(eulers.x, eulers.y, eulers.z); }

    };

    using Quaternion = Quat<float>;
    using vector3 = vec3<float>;

    template <class T = float>
    vec3<T> quat2vec(const Quat<T>& q) { return vec3<T>(q.x, q.y, q.z); };

    template <class T = float>
    Quat<T> vec2quat(const vec3<T>& v) { return Quat<T>(1.0, v.x, v.y, v.z); };

}
