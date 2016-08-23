
#include "Geometry.h"
#include <cmath>
#include <cassert>


std::vector<Vector3D> Geometry::axisRotationMatrix(Vector3D axis, float theta){
    std::vector<Vector3D> result;

    float c = cosf(theta);
    float s = sinf(theta);
    float t = 1.0f - cosf(theta);

    Vector3D col1(t*axis.x*axis.x + c, t*axis.x*axis.y +s*axis.z, t*axis.x*axis.z -s*axis.y);
    Vector3D col2(t*axis.x*axis.y - s*axis.z, t*axis.y*axis.y + c, t*axis.y*axis.z + s*axis.x);
    Vector3D col3(t*axis.x*axis.z +s*axis.y, t*axis.y*axis.z -s*axis.x, t*axis.z*axis.z + c);

    result.push_back(col1);
    result.push_back(col2);
    result.push_back(col3);

    return result;
}

