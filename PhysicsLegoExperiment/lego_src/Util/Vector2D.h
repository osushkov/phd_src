
#ifndef _Vector2D_H_
#define _Vector2D_H_

#include "Matrix.h"

#include <iostream>
#include <cmath>

class Vector2D {

  public:

    Vector2D(float nx, float ny) : x(nx), y(ny) {}
    Vector2D() : x(0.0f), y(0.0f) {}

    inline void print(void) const {
        std::cout << x << " " << y << std::endl;
    }

    inline void scale(float s){
        x *= s;
        y *= s;
    }

    inline float dotProduct(const Vector2D &a) const {
        return x*a.x + y*a.y;
    }

    inline void add(const Vector2D &a){
        x += a.x;
        y += a.y;
    }

    inline float length(void) const {
        return sqrtf(x*x + y*y);
    }

    inline float length2(void) const {
        return (x*x + y*y);
    }

    inline void normalise(void) {
        float inv_length = 1.0f/length();
        scale(inv_length);
    }

    inline void rotate(float theta){
        float nx = cos(theta)*x - sin(theta)*y;
        float ny = sin(theta)*x + cos(theta)*y;
        x = nx;
        y = ny;
    }

    inline Vector2D perpendicular(void) const {
        return Vector2D(y, -x);
    }

    /////////////////////////////////

    float x, y;
};

inline Vector2D operator+ (const Vector2D &a, const Vector2D &b){
    return Vector2D(a.x+b.x, a.y+b.y);
}

inline Vector2D operator- (const Vector2D &a, const Vector2D &b){
    return Vector2D(a.x-b.x, a.y-b.y);
}

inline Vector2D operator* (const Vector2D &a, float s){
    return Vector2D(a.x*s, a.y*s);
}

inline Vector2D operator* (float s, const Vector2D &a){
    return Vector2D(a.x*s, a.y*s);
}

inline Vector2D operator*(const Matrix2 &mat, const Vector2D vec){
    Vector2D result;
    result.x = vec.x*mat(0, 0) + vec.y*mat(0, 1);
    result.y = vec.x*mat(1, 0) + vec.y*mat(1, 1);
    return result;
}

#endif

