
#ifndef _Vector3D_H_
#define _Vector3D_H_

#include "Matrix.h"

#include <iostream>

#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>
#include <cassert>

class Vector3D {

  public:

    Vector3D(float nx, float ny, float nz) : x(nx), y(ny), z(nz) {}
    Vector3D() : x(0.0f), y(0.0f), z(0.0f) {}

    inline void print(void) const {
        std::cout << x << " " << y << " " << z << std::endl;
    }

    inline void scale(float s){
        x *= s;
        y *= s;
        z *= s;
    }

    inline float dotProduct(const Vector3D &a) const {
        return x*a.x + y*a.y + z*a.z;
    }

    inline void add(const Vector3D &a){
        x += a.x;
        y += a.y;
        z += a.z;
    }

    inline float length(void) const {
        return sqrtf(x*x + y*y + z*z);
    }

    inline float length2(void) const {
        return (x*x + y*y + z*z);
    }

    inline void normalise(void) {
        float inv_length = 1.0f/length();
        scale(inv_length);
    }

    inline void rotateX(float theta){
        float nx = x;
        float ny = y*cos(theta) - z*sin(theta);
        float nz = y*sin(theta) + z*cos(theta);
        x = nx;
        y = ny;
        z = nz;
    }

    inline void rotateY(float theta){
        float nx = z*sin(theta) + x*cos(theta);
        float ny = y;
        float nz = z*cos(theta) - x*sin(theta);
        x = nx;
        y = ny;
        z = nz;
    }

    inline void rotateZ(float theta){
        float nx = x*cos(theta) - y*sin(theta);
        float ny = x*sin(theta) + y*cos(theta);
        float nz = z;
        x = nx;
        y = ny;
        z = nz;
    }

    inline Vector3D crossProduct(const Vector3D &a) const {
        Vector3D result;
        result.x = y*a.z - z*a.y;
        result.y = z*a.x - x*a.z;
        result.z = x*a.y - y*a.x;
        return result;
    }

    inline void matrixMultLeft(const std::vector<Vector3D> &columns){
        assert(columns.size() == 3);

        float nx = columns[0].x*x + columns[1].x*y + columns[2].x*z;
        float ny = columns[0].y*x + columns[1].y*y + columns[2].y*z;
        float nz = columns[0].z*x + columns[1].z*y + columns[2].z*z;

        x = nx;
        y = ny;
        z = nz;
    }


    /////////////////////////////////

    float x, y, z;
};


inline Vector3D operator+(const Vector3D &a, const Vector3D &b){
    return Vector3D(a.x+b.x, a.y+b.y, a.z+b.z);
}

inline Vector3D operator-(const Vector3D &a, const Vector3D &b){
    return Vector3D(a.x-b.x, a.y-b.y, a.z-b.z);
}

inline Vector3D operator* (const Vector3D &a, float s){
    return Vector3D(a.x*s, a.y*s, a.z*s);
}

inline Vector3D operator* (float s, const Vector3D &a){
    return Vector3D(a.x*s, a.y*s, a.z*s);
}

inline Vector3D operator*(const Matrix3 &mat, const Vector3D &vec){
    Vector3D result;
    result.x = vec.x*mat(0, 0) + vec.y*mat(0, 1) + vec.z*mat(0, 2);
    result.y = vec.x*mat(1, 0) + vec.y*mat(1, 1) + vec.z*mat(1, 2);
    result.z = vec.x*mat(2, 0) + vec.y*mat(2, 1) + vec.z*mat(2, 2);
    return result;
}

inline Vector3D operator*(const Matrix4 &mat, const Vector3D &vec){
    Vector3D result;
    result.x = vec.x*mat(0, 0) + vec.y*mat(0, 1) + vec.z*mat(0, 2) + mat(0, 3);
    result.y = vec.x*mat(1, 0) + vec.y*mat(1, 1) + vec.z*mat(1, 2) + mat(1, 3);
    result.z = vec.x*mat(2, 0) + vec.y*mat(2, 1) + vec.z*mat(2, 2) + mat(2, 3);
    return result;
}


#endif

