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
    std::vector<float> quaternions;
};

#endif /* TRANSFORM_H_ */
