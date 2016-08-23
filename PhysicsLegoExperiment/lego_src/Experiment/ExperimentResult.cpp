
#include "ExperimentResult.h"
#include "../Util/Geometry.h"

#define _USE_MATH_DEFINES
#include <math.h>


ExperimentResult::ExperimentResult(float orientation, Vector2D position, int label) :
    orientation(orientation), position(position), label(label) {

}

ExperimentResult::ExperimentResult(Transform pose) {
    position = Vector2D(pose.shift.x, pose.shift.y);

    float dp = Vector3D(1.0f, 0.0f, 0.0f).dotProduct(pose.mat * Vector3D(1.0f, 0.0f, 0.0f));
    if(dp <= -1.0f){
        orientation = M_PI;
    }
    else if(dp >= 1.0f){
        orientation = 0.0f;
    }
    else{
        orientation = acosf(dp);
    }

    if(orientation != orientation){
        pose.mat.printOut();
        (pose.mat * Vector3D(1.0f, 0.0f, 0.0f)).print();
        std::cout << dp << " " << orientation << std::endl;

        getchar();
    }

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
    float ori_diff = fabs(fabs(orientation - other.orientation) - 2.0f*M_PI);
    return 3.0f*ori_diff + (position-other.position).length();
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