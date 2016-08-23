
#ifndef _Quaternion_H_
#define _Quaternion_H_

#include "Matrix.h"

class Quaternion {
  public:
    Quaternion();
    Quaternion(float x, float y, float z, float w);
    Quaternion(Matrix3 mat);

    float dotProduct(Quaternion var) const;
    void normalise(void);

    Matrix3 toMatrix(void) const;
    void print(void) const;

    float x, y, z, w;
};

inline Quaternion operator+ (const Quaternion &a, const Quaternion &b){
    return Quaternion(a.x+b.x, a.y+b.y, a.z+b.z, a.w+b.w);
}

#endif
