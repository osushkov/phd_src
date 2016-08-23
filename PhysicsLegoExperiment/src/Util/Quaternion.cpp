
#include "Quaternion.h"
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btMatrix3x3.h>

Quaternion::Quaternion() : x(0.0f), y(0.0f), z(0.0f), w(0.0f) {

}

Quaternion::Quaternion(float x, float y, float z, float w) : 
    x(x), y(y), z(z), w(w) {
    
    //normalise();
}

Quaternion::Quaternion(Matrix3 mat){
    btMatrix3x3 bt_mat(mat(0,0), mat(0,1), mat(0,2),
                       mat(1,0), mat(1,1), mat(1,2),
                       mat(2,0), mat(2,1), mat(2,2));

    btQuaternion result;
    bt_mat.getRotation(result);

    x = result.getX();
    y = result.getY();
    z = result.getZ();
    w = result.getW();
}

float Quaternion::dotProduct(Quaternion var) const {
    return x*var.x + y*var.y + z*var.z + w*var.w;
}

void Quaternion::normalise(void){
    float ilength = 1.0f/sqrtf(x*x + y*y + z*z + w*w);
    x *= ilength;
    y *= ilength;
    z *= ilength;
    w *= ilength;
}

Matrix3 Quaternion::toMatrix(void) const {
    btQuaternion tmp_quaternion(x, y, z, w);
    btMatrix3x3 mat(tmp_quaternion);

    btVector3 col0 = mat.getColumn(0);
    btVector3 col1 = mat.getColumn(1);
    btVector3 col2 = mat.getColumn(2);

    Matrix3 result;
    result(0,0) = col0.getX();
    result(1,0) = col0.getY();
    result(2,0) = col0.getZ();

    result(0,1) = col1.getX();
    result(1,1) = col1.getY();
    result(2,1) = col1.getZ();

    result(0,2) = col2.getX();
    result(1,2) = col2.getY();
    result(2,2) = col2.getZ();

    return result;
}

void Quaternion::print(void) const {
    std::cout << "(" << x << "," << y << "," << z << "," << w << ")" << std::endl;
}
