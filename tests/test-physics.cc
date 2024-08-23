#include <stdio.h>

#include "SHUT/shut.h"
#include "../physics.h"

#include <vector>
#include <array>
#include <stdlib.h>

// Functions exported from MATLAB.
// https://www.codeconvert.ai/matlab-to-c++-converter

std::vector<float> quatmultiply(const std::vector<float>& q, const std::vector<float>& r) {
    std::vector<float> result(4);
    result[0] = q[0] * r[0] - q[1] * r[1] - q[2] * r[2] - q[3] * r[3];
    result[1] = q[0] * r[1] + q[1] * r[0] + q[2] * r[3] - q[3] * r[2];
    result[2] = q[0] * r[2] - q[1] * r[3] + q[2] * r[0] + q[3] * r[1];
    result[3] = q[0] * r[3] + q[1] * r[2] - q[2] * r[1] + q[3] * r[0];
    return result;
}

std::vector<float> cross(const std::vector<float>& A, const std::vector<float>& B) {
    std::vector<float> C(3);
    C[0] = A[1] * B[2] - A[2] * B[1];
    C[1] = A[2] * B[0] - A[0] * B[2];
    C[2] = A[0] * B[1] - A[1] * B[0];
    return C;
}

SHUT_TEST(vec3) {

    SHUT_CASE(Vector3(1.0, 2.0, 3.0) == Vector3(1.0, 2.0, 3.0)); // equality
    SHUT_CASE(Vector3(0.1, 0.2, 0.3) == (Vector3() = Vector3(0.1, 0.2, 0.3))); // assignment

    SHUT_CASE(Vector3(0.2, 0.4, 0.6) == Vector3(0.1, 0.2, 0.3) + Vector3(0.1, 0.2, 0.3)); // addition
    SHUT_CASE(Vector3(0.0, 0.0, 0.0) == Vector3(0.1, 0.2, 0.3) - Vector3(0.1, 0.2, 0.3)); // subtraction
    SHUT_CASE(Vector3(0.2, 0.4, 0.6) == Vector3(0.1, 0.2, 0.3) * 2); // scalar multiplication
    SHUT_CASE(Vector3(0.05, 0.1, 0.15) == Vector3(0.1, 0.2, 0.3) / 2); // scalar division
    
    Vector3 tmp;

    tmp = Vector3(0.0, 0.0, 0.0);
    tmp += Vector3(1.0, 2.0, 3.0);
    SHUT_CASE(Vector3(1.0, 2.0, 3.0) == tmp); // add assign

    tmp = Vector3(0.0, 0.0, 0.0);
    tmp -= Vector3(1.0, 2.0, 3.0);
    SHUT_CASE(Vector3(-1.0, -2.0, -3.0) == tmp); // subtraction assign

    tmp = Vector3(1.0, 2.0, 3.0);
    tmp *= 2.0;
    SHUT_CASE(Vector3(2.0, 4.0, 6.0) == tmp); // multiply assign

    tmp = Vector3(1.0, 2.0, 3.0);
    tmp /= 2.0;
    SHUT_CASE(Vector3(0.5, 1.0, 1.5) == tmp); // divide assign

    SHUT_CASE(Vector3(-1.0, -2.0, -3.0) == -Vector3(1.0, 2.0, 3.0)); // negation
    SHUT_CASE(abs(sqrtf(3*3 + 8*8 + 2*2) - Vector3(3, 8, 2).len()) <= __FLT_EPSILON__); // vector length
    SHUT_CASE(Vector3(3.f / sqrtf(14), 2 / sqrtf(14), -1 / sqrtf(14)) == Vector3(3,2,-1).norm()); // vector normalize

    std::vector<float> a(3);
    std::vector<float> b(3);
    std::vector<float> c(3);
    
    a[0] = 2;
    a[1] = 6;
    a[2] = -1;

    b[0] = 6;
    b[1] = -9;
    b[2] = 3;

    c = cross(a, b);

    SHUT_CASE(Vector3(c[0], c[1], c[2]) == (Vector3(2,6,-1).cross(Vector3(6,-9,3)))); // cross product
    SHUT_CASE(abs((1*1 + 2*2 + 3*3) - Vector3(1,2,3).dot(Vector3(1,2,3))) <= __FLT_EPSILON__) // dot product

}

SHUT_TEST(quat) {

    // Test for equality and assignment
    SHUT_CASE(Quaternion(1.0, 2.0, 3.0, 4.0) == Quaternion(1.0, 2.0, 3.0, 4.0)); // equality
    SHUT_CASE(Quaternion(0.1, 0.2, 0.3, 0.4) == (Quaternion() = Quaternion(0.1, 0.2, 0.3, 0.4))); // assignment

    // Test for addition and subtraction
    SHUT_CASE(Quaternion(0.2, 0.4, 0.6, 0.8) == Quaternion(0.1, 0.2, 0.3, 0.4) + Quaternion(0.1, 0.2, 0.3, 0.4)); // addition
    SHUT_CASE(Quaternion(0.0, 0.0, 0.0, 0.0) == Quaternion(0.1, 0.2, 0.3, 0.4) - Quaternion(0.1, 0.2, 0.3, 0.4)); // subtraction

    // Test for scalar multiplication and division
    SHUT_CASE(Quaternion(0.2, 0.4, 0.6, 0.8) == Quaternion(0.1, 0.2, 0.3, 0.4) * 2); // scalar multiplication
    SHUT_CASE(Quaternion(0.05, 0.1, 0.15, 0.2) == Quaternion(0.1, 0.2, 0.3, 0.4) / 2); // scalar division

    // Test for add-assign, subtract-assign, multiply-assign, and divide-assign
    Quaternion tmp;

    tmp = Quaternion(0.0, 0.0, 0.0, 0.0);
    tmp += Quaternion(1.0, 2.0, 3.0, 4.0);
    SHUT_CASE(Quaternion(1.0, 2.0, 3.0, 4.0) == tmp); // add assign

    tmp = Quaternion(1.0, 2.0, 3.0, 4.0);
    tmp -= Quaternion(1.0, 2.0, 3.0, 4.0);
    SHUT_CASE(Quaternion(0.0, 0.0, 0.0, 0.0) == tmp); // subtract assign

    tmp = Quaternion(1.0, 2.0, 3.0, 4.0);
    tmp *= 2.0;
    SHUT_CASE(Quaternion(2.0, 4.0, 6.0, 8.0) == tmp); // multiply assign

    tmp = Quaternion(2.0, 4.0, 6.0, 8.0);
    tmp /= 2.0;
    SHUT_CASE(Quaternion(1.0, 2.0, 3.0, 4.0) == tmp); // divide assign

    // Test for negation
    SHUT_CASE(Quaternion(-1.0, -2.0, -3.0, -4.0) == -Quaternion(1.0, 2.0, 3.0, 4.0)); // negation

    // Test for quaternion multiplication
    Quaternion q1(1.0, 2.0, 3.0, 4.0);
    Quaternion q2(2.0, 3.0, 4.0, 5.0);
    SHUT_CASE(Quaternion(-36.0, 6.0, 12.0, 12.0) == (q1 * q2)); // quaternion multiplication

    // Test for quaternion normalization
    q1 = Quaternion(1.0, 2.0, 3.0, 4.0);
    SHUT_CASE(abs(sqrtf(1.0*1.0 + 2.0*2.0 + 3.0*3.0 + 4.0*4.0) - q1.len()) <= __FLT_EPSILON__); // quaternion length
    Quaternion norm_q1 = q1.norm();
    float len_norm_q1 = norm_q1.len();
    SHUT_CASE(abs(1.0f - len_norm_q1) <= __FLT_EPSILON__); // normalized quaternion length should be 1

    // Test for quaternion rotation (rotate another quaternion)
    Quaternion q3 = q1.rotate(q2);
    Quaternion expected_rotation = q1 * q2 * Quaternion(q1.w, -q1.x, -q1.y, -q1.z);
    SHUT_CASE(expected_rotation == q3); // quaternion rotation

    // Test for axis-angle conversion
    Vector3 axis(1.0f, 0.0f, 0.0f);
    Quaternion axis_angle_q = Quaternion().from_axis_angle(M_PI / 2.0f, axis);
    SHUT_CASE(abs(axis_angle_q.w - cosf(M_PI / 4.0f)) <= __FLT_EPSILON__); // check w component
    SHUT_CASE(abs(axis_angle_q.x - sinf(M_PI / 4.0f)) <= __FLT_EPSILON__); // check x component
    SHUT_CASE(axis_angle_q.y == 0.0f && axis_angle_q.z == 0.0f); // check y and z components
    
}

int main() {
    SHUT_RUN_TEST(vec3);
    SHUT_RUN_TEST(quat);
    SHUT_TEST_INFO();
}