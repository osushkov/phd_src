/*
 * CentralisePoints.cpp
 *
 *  Created on: 29/10/2009
 *      Author: osushkov
 */

#include "CentralisePoints.h"
#include "../OptimiserClass/Optimiser.h"
#include "../OptimiserClass/RRNelderMead.h"
#include "../OptimiserClass/NelderMead.h"
#include "../OptimiserClass/ParticleSwarm.h"
#include "../OptimiserClass/GeneticAlgorithm.h"
#include "../OptimiserClass/SimulatedAnnealing.h"
#include "../Util/Matrix.h"
#include "../Util/Geometry.h"

#include <vector>
#include <algorithm>
#include <limits.h>

static Transform paramsToTransform(const std::vector<float> &params){
    /*
    for(unsigned i = 0; i < params.size(); i++){
        assert(params[i] >= 0.0f && params[i] <= 1.0f);
    }
    */

    Transform result;

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

    result.mat(0, 0) = q0*q0 + q1*q1 - q2*q2 - q3*q3;
    result.mat(0, 1) = 2.0f*(q1*q2 + q0*q3);
    result.mat(0, 2) = 2.0f*(q1*q3 - q0*q2);

    result.mat(1, 0) = 2.0f*(q1*q2 - q0*q3);
    result.mat(1, 1) = q0*q0 - q1*q1 + q2*q2 - q3*q3;
    result.mat(1, 2) = 2.0f*(q2*q3 + q0*q1);

    result.mat(2, 0) = 2.0f*(q1*q3 + q0*q2);
    result.mat(2, 1) = 2.0f*(q2*q3 - q0*q1);
    result.mat(2, 2) = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    /*result.shift.x = (params[4]-0.5f)*100.0f;
    result.shift.y = (params[5]-0.5f)*100.0f;
    result.shift.z = (params[6]-0.5f)*100.0f;
*/
    return result;
}



CentralisePointsFitFunction::CentralisePointsFitFunction(const std::vector<Vector3D> &points) :
    points(points) {

}

float CentralisePointsFitFunction::eval(std::vector<float> &params, float iter_frac){
    Transform transform = paramsToTransform(params);

    float result = 0.0f;

    float minx = 0.0f, maxx = 0.0f;
    float miny = 0.0f, maxy = 0.0f;
    float minz = 0.0f, maxz = 0.0f;

    float alpha = 0.01f;

    for(unsigned i = 0; i < points.size(); i++){
        Vector3D tp = applyTransform(transform, points[i]);

        if(i == 0 || tp.x < minx){ minx = (1.0f-alpha)*minx + alpha*tp.x; }
        if(i == 0 || tp.y < miny){ miny = (1.0f-alpha)*miny + alpha*tp.y; }
        if(i == 0 || tp.z < minz){ minz = (1.0f-alpha)*minz + alpha*tp.z; }

        if(i == 0 || tp.x > maxx){ maxx = (1.0f-alpha)*maxx + alpha*tp.x; }
        if(i == 0 || tp.y > maxy){ maxy = (1.0f-alpha)*maxy + alpha*tp.y; }
        if(i == 0 || tp.z > maxz){ maxz = (1.0f-alpha)*maxz + alpha*tp.z; }

    }

    result = (maxx - minx)*(maxy - miny)*(maxz - minz);

    return result;
}

Vector3D CentralisePointsFitFunction::applyTransform(const Transform &t, const Vector3D &vec){
    Vector3D result = vec;// + t.shift;
    result = t.mat*result;
    return result;
}


CentralisePoints::CentralisePoints(){

}

CentralisePoints::~CentralisePoints(){

}

float CentralisePoints::calculateTransform(const std::vector<Vector3D> &points, Transform &result){
    CentralisePointsFitFunction *func = new CentralisePointsFitFunction(points);

    float best_fitness = 0.0f;
    std::vector<float> best_params;

    for(unsigned i = 0; i < 50; i++){
        std::vector<float> start_params;
        for(unsigned j = 0; j < 4; j++){
            start_params.push_back((float)rand()/(float)RAND_MAX);
        }

        NelderMead optimiser(start_params);
        std::vector<float> optimised_result = optimiser.optimise(func, 4, 100);
        float cur_fitness = func->eval(optimised_result, 1.0f);

        if(i == 0 || cur_fitness < best_fitness){
            best_fitness = cur_fitness;
            best_params = optimised_result;
            std::cout << "new best: " << best_fitness << std::endl;
        }
    }

    NelderMead optimiser(best_params);
    best_params = optimiser.optimise(func, 4, 500);

    result = paramsToTransform(best_params);
    return func->eval(best_params, 1.0f);

    /*
    ParticleSwarm optimiser(1000);
    std::vector<float> optimised_result = optimiser.optimise(func, 4, 500);

    NelderMead optimiser2(optimised_result);
    optimised_result = optimiser2.optimise(func, 4, 1000);

    result = paramsToTransform(optimised_result);
    return func->eval(optimised_result);
    */
}

