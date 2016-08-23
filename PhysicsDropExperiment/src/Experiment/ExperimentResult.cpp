
#include "ExperimentResult.h"
#include "../Util/Geometry.h"
#include <cmath>

ExperimentResult::ExperimentResult(Vector3D orientation, int label):
    orientation(orientation), label(label) {

}

ExperimentResult::ExperimentResult(Transform pose) {
    Matrix3 rmat_inv;
    rmat_inv.isTranspose(pose.mat);
    orientation = rmat_inv * Vector3D(0.0f, 0.0f, 1.0f);

    label = -1;
}

ExperimentResult::~ExperimentResult(){

}

Vector3D ExperimentResult::getOrientation(void) const {
    return orientation;
}

float ExperimentResult::distance(const ExperimentResult &other) const {
    float dp = other.orientation.dotProduct(orientation);
    if(dp > 1.0f){ dp = 1.0f; }
    if(dp < -1.0f){ dp = -1.0f; }
    return fabs(acosf(dp));
}

int ExperimentResult::getLabel(void) const {
    return label;
}

void ExperimentResult::setLabel(int new_label){
    label = new_label;
}

void ExperimentResult::load(std::ifstream &in_file){
    in_file >> orientation.x;
    in_file >> orientation.y;
    in_file >> orientation.z;
    in_file >> label;
}

void ExperimentResult::save(std::ofstream &out_file){
    out_file << orientation.x << std::endl;
    out_file << orientation.y << std::endl;
    out_file << orientation.z << std::endl;
    out_file << label << std::endl;
}