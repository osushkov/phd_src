/*
 * SceneBestFit.cpp
 *
 *  Created on: 07/01/2010
 *      Author: osushkov
 */

#include "SceneBestFit.h"
#include "OptimiserClass/Optimiser.h"
#include "OptimiserClass/NelderMead.h"
#include "OptimiserClass/ParticleSwarm.h"
#include "OptimiserClass/RRNelderMead.h"
#include "Util/Matrix.h"
#include "Util/Geometry.h"
#include "Util/Timer.h"

#include <vector>
#include <algorithm>

static Transform paramsToTransform(const std::vector<float> &params){
    float q0, q1, q2, q3;

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

    result.quaternion = Quaternion(q0, q1, q2, q3);
    result.mat = result.quaternion.toMatrix();

    result.shift.x = (params[4]-0.5f)*100.0f; //(params[4]-0.5f)*100.0f;
    result.shift.y = (params[5])*100.0f; //(params[5]-0.5f)*100.0f;
    result.shift.z = (params[6])*100.0f; //(params[6]-0.5f)*100.0f;


    return result;
}

static std::vector<float> transformToParams(const Transform &transform){
    std::vector<float> result;

    result.push_back(transform.quaternion.x/2.0f + 0.5f);
    result.push_back(transform.quaternion.y/2.0f + 0.5f);
    result.push_back(transform.quaternion.z/2.0f + 0.5f);
    result.push_back(transform.quaternion.w/2.0f + 0.5f);

    result.push_back(transform.shift.x/100.0f + 0.5f);
    result.push_back(transform.shift.y/100.0f);
    result.push_back(transform.shift.z/100.0f);

    return result;
}

SceneBestFitFunction::SceneBestFitFunction
(const std::vector<BestFitFeaturePair> &matching_features) :
    matching_features(matching_features) {

}

float SceneBestFitFunction::eval(std::vector<float> &params){
    Transform t = paramsToTransform(params);

    float result = 0.0f;
    std::vector<float> ordered;

    for(unsigned i = 0; i < matching_features.size(); i++){
        Vector3D p1 = matching_features[i].fpos0;
        Vector3D p2 = matching_features[i].fpos1;

        p2 = applyTransform(t, p2);
        float d = (p1-p2).length2();
        ordered.push_back(d);
        //result += d;
    }

    sort(ordered.begin(), ordered.end());
    unsigned num = ordered.size();
    for(unsigned i = 0; i < num; i++){
        result += ordered[i];
    }
  
    result /= (float)num;
    return result;
}

Vector3D SceneBestFitFunction::applyTransform(const Transform &t, const Vector3D &vec){
    return t.mat*vec + t.shift;
}


SceneBestFit::SceneBestFit() : use_local(false) {

}
SceneBestFit::~SceneBestFit(){
}

float SceneBestFit::calculateBestFit(const std::vector<BestFitFeaturePair> &matching_features,
                                     Transform &result, bool use_prev){

    SceneBestFitFunction *func = new SceneBestFitFunction(matching_features);

    NelderMead optimiser(transformToParams(hint_transform));
    std::vector<float> optimised_result = optimiser.optimise(func, 7, 10000);

    result = paramsToTransform(optimised_result);
    prev = optimised_result;
    use_local = true;

    return func->eval(optimised_result);
}

void SceneBestFit::reset(void){
    use_local = false;
    prev = std::vector<float>();
}

