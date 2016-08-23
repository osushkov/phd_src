
#include "ExperimentResult.h"
#include "../Util/Geometry.h"

#define _USE_MATH_DEFINES
#include <math.h>


ExperimentResult::ExperimentResult(float orientation, Vector2D position, int label) :
    orientation(orientation), position(position), label(label) {

}

ExperimentResult::ExperimentResult(Transform pose) {
    position = Vector2D(pose.shift.x, pose.shift.y);

    Vector3D v = pose.mat * Vector3D(1.0f, 0.0f, 0.0f);
    orientation = atan2f(v.y, v.x);

    while(orientation < -M_PI){
        orientation += 2.0f*M_PI;
    }

    while(orientation > M_PI){
        orientation -= 2.0f*M_PI;
    }

    label = -1;
}

ExperimentResult::ExperimentResult(){
    label = -1;
}

ExperimentResult::~ExperimentResult(){

}

float ExperimentResult::getOrientation(void) const {
    return orientation;
}

Vector2D ExperimentResult::getPosition(void) const {
    return position;
}

float ExperimentResult::distance(const ExperimentResult &other) const {
    float ori_diff = orientation - other.orientation;
    while(ori_diff < -M_PI){
        ori_diff += 2.0f*M_PI;
    }

    while(ori_diff > M_PI){
        ori_diff -= 2.0f*M_PI;
    }

    ori_diff = fabs(ori_diff);
    return 6.0f*ori_diff + (position-other.position).length();
}

int ExperimentResult::getLabel(void) const {
    return label;
}

void ExperimentResult::setLabel(int new_label){
    label = new_label;
}

void ExperimentResult::load(std::ifstream &in_file){
    in_file >> orientation;
    in_file >> position.x;
    in_file >> position.y;
    in_file >> label;
}

void ExperimentResult::save(std::ofstream &out_file){
    out_file << orientation << std::endl;
    out_file << position.x << std::endl;
    out_file << position.y << std::endl;
    out_file << label << std::endl;
}