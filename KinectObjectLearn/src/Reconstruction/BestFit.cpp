/*
 * BestFit.cpp
 *
 *  Created on: 07/07/2009
 *      Author: osushkov
 */

#include "BestFit.h"
#include "../OptimiserClass/Optimiser.h"
#include "../OptimiserClass/RRNelderMead.h"
#include "../OptimiserClass/NelderMead.h"
#include "../OptimiserClass/ParticleSwarm.h"
#include "../OptimiserClass/GeneticAlgorithm.h"
#include "../Util/Matrix.h"
#include "../Util/Geometry.h"
#include "../Util/Timer.h"
#include "../Util/Common.h"

#include <vector>
#include <algorithm>

static Vector3D applyTransform(const Transform &t, const Vector3D &vec){
    return t.mat*vec + t.shift;
}

static Transform paramsToTransform(const std::vector<float> &params){
    float q0, q1, q2, q3;

    if(params.size() != 7){
        std::cerr << "params incorrect size: " << params.size() << std::endl;
    }
    q0 = (params[0]-0.5f)*2.0f;
    q1 = (params[1]-0.5f)*2.0f;
    q2 = (params[2]-0.5f)*2.0f;
    q3 = (params[3]-0.5f)*2.0f;


    float l = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= l;
    q1 /= l;
    q2 /= l;
    q3 /= l;


    Transform result;

    result.quaternions.push_back(q0);
    result.quaternions.push_back(q1);
    result.quaternions.push_back(q2);
    result.quaternions.push_back(q3);

    result.mat = Common::quaternionsToMatrix(result.quaternions);

    result.shift.x = (params[4]-0.5f)*40.0f; //(params[4]-0.5f)*100.0f;
    result.shift.y = (params[5]-0.5f)*40.0f;      //(params[5]-0.5f)*100.0f;
    result.shift.z = (params[6]-0.5f)*40.0f; //(params[6]-0.5f)*100.0f;

    result.secondary_shift = Vector3D(0.0f, 0.0f, 0.0f);

    return result;
}

static std::vector<float> transformToParams(const Transform &transform){
    std::vector<float> result;

    result.push_back(transform.quaternions[0]/2.0f + 0.5f);
    result.push_back(transform.quaternions[1]/2.0f + 0.5f);
    result.push_back(transform.quaternions[2]/2.0f + 0.5f);
    result.push_back(transform.quaternions[3]/2.0f + 0.5f);

    Vector3D shift = transform.shift;

    result.push_back(shift.x/40.0f + 0.5f);
    result.push_back(shift.y/40.0f + 0.5f);
    result.push_back(shift.z/40.0f + 0.5f);

    return result;
}

BestFitFunction::BestFitFunction(std::vector<BestFitFeaturePair> matching_features) :
    matching_features(matching_features) {
    assert(matching_features.size() >= 3);

}

float BestFitFunction::eval(std::vector<float> &params){
    Transform t = paramsToTransform(params);
    float result = 0.0f;

    for(unsigned i = 0; i < matching_features.size(); i++){
        Vector3D p1 = matching_features[i].fpos0;
        Vector3D p2 = applyTransform(t, matching_features[i].fpos1);

        float d = (p1-p2).length();// * matching_features[i].pmatch;
        result += d*matching_features[i].pmatch;
    }

    assert(matching_features.size() > 0);
    result /= (float)matching_features.size();

    return result;
}


BestFit::BestFit() {
    hint_shift = Vector3D(0.0f, 0.0f, 0.0f);
}

BestFit::~BestFit(){
}

float BestFit::calculateBestFit(const std::vector<BestFitFeaturePair> &matching_features,
                                Transform &result, bool use_prev){

    Transform identity_transform;
    identity_transform.mat.identity();
    identity_transform.quaternions = std::vector<float>(4, 0.0f);
    identity_transform.quaternions[0] = 1.0f;
    identity_transform.shift = identity_transform.secondary_shift = Vector3D(0.0f, 0.0f, 0.0f);
    
    BestFitFunction *func = new BestFitFunction(matching_features);
    NelderMead optimiser(transformToParams(identity_transform));
    std::vector<float> optimised_result = optimiser.optimise(func, 7, 1000);
    result = paramsToTransform(optimised_result);
    float error = func->eval(optimised_result);

    delete func;
    return error;
}

void BestFit::setHintShift(Vector3D hint){
    hint_shift = hint;
}
