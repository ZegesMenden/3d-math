#include <vectorMath.h>
#include <Arduino.h>

vector3 & vector3::operator+=(const vector3& v){
    x += v.x;
    y += v.y;
    z += v.z;
    return(*this);    
}

vector3 & vector3::operator-=(const vector3& v){
    x -= v.x;
    y -= v.y;
    z -= v.z;
    return(*this);    
}

vector3 & vector3::operator*=(const vector3& v){
    x *= v.x;
    y *= v.y;
    z *= v.z;
    return(*this);
}

vector3 & vector3::operator*=(float scale){
    x *= scale;
    y *= scale;
    z *= scale;
    return(*this);
}

vector3 & vector3::operator/=(const vector3& v){
    x /= v.x;
    y /= v.y;
    z /= v.z;
    return(*this);
}

vector3 & vector3::operator/=(float scale){
    x /= scale;
    y /= scale;
    z /= scale;
    return(*this);
}

vector3 & vector3::norm(){
    float normal = sqrt( x * x + y * y + z * z);

    x /= normal;
    y /= normal;
    z /= normal;

    return(*this);

}

float vector3::len() {
    return( sqrt( x * x + y * y + z * z) );
}