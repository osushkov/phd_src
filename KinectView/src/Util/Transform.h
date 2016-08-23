/*
 * Transform.h
 *
 *  Created on: 14/12/2009
 *      Author: osushkov
 */

#ifndef TRANSFORM_H_
#define TRANSFORM_H_

#include "Matrix.h"
#include "Vector3D.h"
#include <vector>

struct Transform {
    Matrix3 mat;
    Vector3D shift;
    Vector3D secondary_shift;
};

inline Transform identityTransform(void){
    Transform result;
    result.shift = Vector3D(0.0f, 0.0f, 0.0f);
    result.mat.identity();
    return result;
}

#endif /* TRANSFORM_H_ */
