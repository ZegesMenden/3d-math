#pragma once

// for sqrtf, sinf, cosf, atan2f
#include <math.h>

class Quaternion;

class Vector3 {
// for sqrtf, sinf, cosf, atan2f
#include <math.h>

class Quaternion;

class Vector3 {
public:

    float x, y, z;

    Vector3() : x(0), y(0), z(0) {};
    Vector3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {};
    Vector3(const Vector3 &other) { *this = other; }
    inline Vector3(const Quaternion &other);

    bool operator==(const Vector3 &other) {
        return (abs(x - other.x) < 1e-12) && (abs(y - other.y) < 1e-12) && (abs(z - other.z) < 1e-12);
    }

    Vector3& operator=(const Vector3 &other) {
        x = other.x;
        y = other.y;
        z = other.z;
        return *this;
    }

    Vector3 operator+(const Vector3 &other) const {
        return Vector3(x+other.x, y+other.y, z+other.z);
    }

    Vector3& operator+=(const Vector3 &other) {
        *this = *this + other;
        return *this;
    }

    Vector3 operator-(const Vector3 &other) const {
        return Vector3(x-other.x, y-other.y, z-other.z);
    }

    Vector3& operator-=(const Vector3 &other) {
        *this = *this - other;
        return *this;
    }

    Vector3 operator-() const {
        return Vector3(-x, -y, -z);
    }

    Vector3 operator*(const float scalar) const {
        return Vector3(x*scalar, y*scalar, z*scalar);
    }

    Vector3& operator*=(const float scalar) {
        *this = *this * scalar;
        return *this;
    }

    Vector3 operator/(const float scalar) const {
        return Vector3(x/scalar, y/scalar, z/scalar);
    }

    Vector3& operator/=(const float scalar) {
        *this = *this / scalar;
        return *this;
    }

    Vector3 norm() const {
        return Vector3(*this) / len();
    }

    /**
	 * @brief Returns the cross product of the left and right hand vectors
	 * @return cross product of the two vectors
	 */
	Vector3 cross(const Vector3& v) const {
		Vector3 ret(   this->y * v.z - this->z * v.y, 
                    this->z * v.x - this->x * v.z, 
                    this->x * v.y - this->y * v.x);
		return ret;
    }

    /**
	 * @brief Returns the dot product of the left and right hand vectors
	 * @return dot product of the two vectors
	 */
	float dot(const Vector3& other) const {
		return (x*other.x + y*other.y + z*other.z);
	}

    float len() const {
        return sqrtf(x*x + y*y + z*z);
    }

    float angle_between_vectors(const Vector3 &other) const {
		return acosf(dot(other) / (len() * (other.len())));
	}
    /**
	 * @brief Returns the dot product of the left and right hand vectors
	 * @return dot product of the two vectors
	 */
	float dot(const Vector3& other) const {
		return (x*other.x + y*other.y + z*other.z);
	}

    float len() const {
        return sqrtf(x*x + y*y + z*z);
    }

    float angle_between_vectors(const Vector3 &other) const {
		return acosf(dot(other) / (len() * (other.len())));
	}

};

class Quaternion {
class Quaternion {
public:

    float w, x, y, z;

    Quaternion() {w=0; x=0; y=0; z=0;};
    Quaternion(float _w, float _x, float _y, float _z): w(_w), x(_x), y(_y), z(_z) {};
    Quaternion(const Quaternion &other) { *this = other; };
    Quaternion(const Vector3 &other) : w(0), x(other.x), y(other.y), z(other.z) {};

    bool operator==(const Quaternion &other) {
        return (abs(w - other.w) < __FLT_EPSILON__) && (abs(x - other.x) < __FLT_EPSILON__) && (abs(y - other.y) < __FLT_EPSILON__) && (abs(z - other.z) < __FLT_EPSILON__);
    }

    Quaternion& operator=(const Quaternion &other) {

    float w, x, y, z;

    Quaternion() {w=0; x=0; y=0; z=0;};
    Quaternion(float _w, float _x, float _y, float _z): w(_w), x(_x), y(_y), z(_z) {};
    Quaternion(const Quaternion &other) { *this = other; };
    Quaternion(const Vector3 &other) : w(0), x(other.x), y(other.y), z(other.z) {};

    bool operator==(const Quaternion &other) {
        return (abs(w - other.w) < __FLT_EPSILON__) && (abs(x - other.x) < __FLT_EPSILON__) && (abs(y - other.y) < __FLT_EPSILON__) && (abs(z - other.z) < __FLT_EPSILON__);
    }

    Quaternion& operator=(const Quaternion &other) {
        w = other.w;
        x = other.x;
        y = other.y;
        z = other.z;
        return *this;
    }

    Quaternion operator+(const Quaternion &other) const {
        return Quaternion(w+other.w, x+other.x, y+other.y, z+other.z);
    }

    Quaternion& operator+=(const Quaternion &other) {
        *this = *this + other;
        return *this;
    }

    Quaternion operator-(const Quaternion &other) const {
        return Quaternion(w-other.w, x-other.x, y-other.y, z-other.z);
    }

    Quaternion& operator-=(const Quaternion &other) {
        *this = *this - other;
        return *this;
    }

    Quaternion operator-() const {
        return Quaternion(-w, -x, -y, -z);
    }
    Quaternion operator+(const Quaternion &other) const {
        return Quaternion(w+other.w, x+other.x, y+other.y, z+other.z);
    }

    Quaternion& operator+=(const Quaternion &other) {
        *this = *this + other;
        return *this;
    }

    Quaternion operator-(const Quaternion &other) const {
        return Quaternion(w-other.w, x-other.x, y-other.y, z-other.z);
    }

    Quaternion& operator-=(const Quaternion &other) {
        *this = *this - other;
        return *this;
    }

    Quaternion operator-() const {
        return Quaternion(-w, -x, -y, -z);
    }

    Quaternion operator*(const float scalar) const {
        return Quaternion(w*scalar, x*scalar, y*scalar, z*scalar);
    }
    Quaternion operator*(const float scalar) const {
        return Quaternion(w*scalar, x*scalar, y*scalar, z*scalar);
    }

    Quaternion& operator*=(const float scalar) {
        *this = *this * scalar;
        return *this;
    }

    Quaternion operator/(const float scalar) const {
        return Quaternion(w/scalar, x/scalar, y/scalar, z/scalar);
    }

    Quaternion conjugate() {
        return Quaternion(w, -x, -y, -z);
    }

    Quaternion& operator/=(const float scalar) {
        *this = *this / scalar;
        return *this;
    }

    // a*e - b*f - c*g - d*h
    // b*e + a*f + c*h - d*g
    // a*g - b*h + c*e + d*f
    // a*h + b*g - c*f + d*e
    Quaternion operator*(const Quaternion &other) const {
        Quaternion ret;
        ret.w = w*other.w - x*other.x - y*other.y - z*other.z;
        ret.x = x*other.w + w*other.x + y*other.z - z*other.y;
        ret.y = w*other.y - x*other.z + y*other.w + z*other.x;
        ret.z = w*other.z + x*other.y - y*other.x + z*other.w;
        return ret;
    }

    /**
	 * @brief Computes the quaternion from a set of axis angles and magnitude
	 * @param t magnitude
	 * @param ang normalized axis angles
	 * @return quaternion
	 */
	Quaternion from_axis_angle(const float t, const Vector3& ang) const {
        Quaternion ret;
        float s = sinf(t / 2.0f);

		ret.w = cosf(t / 2.0f);
		ret.x = ang.x * s;
		ret.y = ang.y * s;
		ret.z = ang.z * s;

		return ret;
	};

	/**
	 * @brief rotates another quaternion by this quaternion
	 * @param q quaternion to rotate
	 * @return rotated quaternion as a quaternion
	 */
	Quaternion rotate(const Quaternion& q) const {
		Quaternion ret = (*this * q) * Quaternion(w, -x, -y, -z);
		return ret;
	};

    /**
	 * @brief rotates another quaternion by this quaternion
	 * @param q quaternion to rotate
	 * @return rotated quaternion as a quaternion
	 */
	Vector3 rotate(const Vector3& v) const {
		Vector3 ret = ((*this * v) * Quaternion(w, -x, -y, -z));
		return ret;
	};

    Quaternion norm() const {
        return Quaternion(*this) / len();
    }

    float len() const {
        return sqrtf(w*w + x*x + y*y + z*z);
    }

	const Vector3 euler_angles() const {

		float roll = atan2f(2.0f * (w*x + y*z), 1.0f - 2.0f * (x*x + y*y));
		float pitch = asinf(2.0f * (w*y - x*z));
		float yaw = atan2f(2.0f * (w*z + x*y), 1.0f - 2.0f * (y*y + z*z));

		// T r = atan2f(w * x + y * z, 0.5f - x * x - y * y);
		// T p = asinf(-2.0f * (x * z - w * y));
		// T y = atan2f(w * z + x * y, 0.5f - y * y - z * z);

		Vector3 ret = Vector3(roll, pitch, yaw);
		return ret;
	};

};

inline Vector3::Vector3(const Quaternion &other) {
    // Convert quaternion to vector
    x = other.x;
    y = other.y;
    z = other.z;
}
