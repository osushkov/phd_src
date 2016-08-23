/*
 * ObjectPoseFilter.cpp
 *
 *  Created on: 11/12/2009
 *      Author: osushkov
 */

#include "ObjectPoseFilter.h"

ObjectPoseFilter::ObjectPoseFilter() {
    reset();
}

ObjectPoseFilter::~ObjectPoseFilter() {

}

void ObjectPoseFilter::performMotionUpdate(void) {
    for(unsigned i = 0; i < 7; i++){
        current_covariance(i,i) += process_noise(i,0);
    }
}

void ObjectPoseFilter::performMeasurementUpdate(Matrix<float,7,1> measurement,
                                                Matrix<float,7,1> measurement_noise){

    Matrix<float,7,7> tmp0 = current_covariance;
    for(unsigned i = 0; i < 7; i++){
        tmp0(i,i) += measurement_noise(i, 0);
    }

    Matrix<float,7,7> tmp1;
    tmp1.isInverse(tmp0);

    Matrix<float,7,7> kalman_gain;
    kalman_gain.isMult(current_covariance,tmp1);

    Matrix<float,7,1> to_vector;
    for(unsigned i = 0; i < 7; i++){
        to_vector(i,0) = measurement(i,0) - current_pose(i,0);
    }

    Matrix<float,7,1> innovation_vector;
    innovation_vector.isMult(kalman_gain, to_vector);

    for(unsigned i = 0; i < 7; i++){
        current_pose(i,0) += innovation_vector(i,0);
    }

    Matrix<float,7,7> identity;
    identity.identity();

    identity -= kalman_gain;

    Matrix<float,7,7> new_covariance;
    new_covariance.isMult(identity, current_covariance);

    current_covariance = new_covariance;
}

void ObjectPoseFilter::reset(void){
    current_pose.reset();
    for(unsigned i = 0; i < 3; i++){
        current_pose(i, 0) = 30.0f;
    }

    current_pose(3, 0) = 1.0f;
    current_pose(4, 0) = 0.0f;
    current_pose(5, 0) = 0.0f;
    current_pose(6, 0) = 0.0f;


    current_covariance.reset();
    for(unsigned i = 0; i < 3; i++){
        current_covariance(i, i) = 20.0f*20.0f;
        process_noise(i,0) = 5.0f;
    }
    for(unsigned i = 3; i < 7; i++){
        current_covariance(i, i) = 1.0f*1.0f;
        process_noise(i,0) = 0.1f;
    }
}