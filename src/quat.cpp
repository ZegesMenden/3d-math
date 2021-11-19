#include <quat.h>
#include <vector.h>
#include <arduino.h>

Quaternion & Quaternion::operator*=(const Quaternion &other)
{
    Quaternion qNew;

    qNew.w = ( this->w * other.w ) - ( this->x * other.x ) - ( this->y * other.y ) - ( this->z * other.z );
    qNew.x = ( this->w * other.x ) + ( this->x * other.w ) + ( this->y * other.z ) - ( this->z * other.y );
    qNew.y = ( this->w * other.y ) - ( this->x * other.z ) + ( this->y * other.w ) + ( this->z * other.x );
    qNew.z = ( this->w * other.z ) + ( this->x * other.y ) - ( this->y * other.x ) + ( this->z * other.w );

    return (*this = qNew);
};

vector3 Quaternion::rotate(const vector3 &v) const
{
    Quaternion q(0, v.x, v.y, v.z);
    Quaternion qNew = *this * q * Quaternion(this->w, -this->x, -this->y, -this->z);

    return vector3(qNew.x, qNew.y, qNew.z);
}

Quaternion Quaternion::rotate(const Quaternion &q) const
{
    Quaternion qNew = *this * q * Quaternion(this->w, -this->x, -this->y, -this->z);

    return qNew;
}

Quaternion quaternionFromEuler(float roll, float pitch, float yaw)
{
    float cr = cos( roll / 2 );
    float cp = cos( pitch / 2 );
    float cy = cos( yaw / 2 );

    float sr = sin( roll / 2 );
    float sp = sin( pitch / 2 );
    float sy = sin( yaw / 2 );

    Quaternion qNew;

    qNew.w = cr * cp * cy + sr * sp * sy;
    qNew.x = sr * cp * cy - cr * sp * sy;
    qNew.y = cr * sp * cy + sr * cp * sy;
    qNew.z = cr * cp * sy - sr * sp * cy;

    return qNew;
}

vector3 quaternionToEuler(const Quaternion &q)
{
    float r = atan2( 2.0 * ( q.w * q.x + q.y * q.z ), 1.0 - 2.0 * ( q.x * q.x + q.y * q.y ) );
    float p = 2.0 * ( q.w * q.y - q.z * q.x );
    float y = atan2( 2.0 * ( q.w * q.z + q.x * q.y ), 1.0 - 2.0 * ( q.y * q.y + q.z * q.z ) );
    
    return vector3(r, p, y);
}

Quaternion Quaternion::from_axis_angle(float t, vector3 &eulerAngles)
{
    float sn = sin(t/2.0);

    w = cos(t/2.0);
    x *= sn;
    y *= sn;
    z *= sn;

    return(*this);
}

Quaternion Quaternion::fractional(float alpha)
{
    w = 1-alpha + alpha*w;
    x *= alpha;
    y *= alpha;
    z *= alpha;
    return normalize();
}

Quaternion Quaternion::rotation_between_vectors(const vector3 &v)
{
    Quaternion q = *this * Quaternion(0, v.x, v.y, v.z);
    q.w = 1 - q.w;
    return(q.normalize());
}