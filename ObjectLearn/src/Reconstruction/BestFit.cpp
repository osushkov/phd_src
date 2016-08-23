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

#include <vector>
#include <algorithm>

static Vector3D applyTransform(const Transform &t, const Vector3D &vec){
    return t.mat*(vec - t.shift) + t.shift;
}

static Transform paramsToTransform(const std::vector<float> &params, Vector3D region_centre){
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

    result.quaternions.push_back(q0);
    result.quaternions.push_back(q1);
    result.quaternions.push_back(q2);
    result.quaternions.push_back(q3);

    result.mat = Common::quaternionsToMatrix(result.quaternions);

    result.shift.x = (params[4]-0.5f)*40.0f; //(params[4]-0.5f)*100.0f;
    result.shift.y = (params[5]-0.5f)*40.0f;      //(params[5]-0.5f)*100.0f;
    result.shift.z = (params[6]-0.5f)*40.0f; //(params[6]-0.5f)*100.0f;

    result.shift = result.shift + region_centre;

    return result;
}

static std::vector<float> transformToParams(const Transform &transform, Vector3D region_centre){
    std::vector<float> result;

    result.push_back(transform.quaternions[0]/2.0f + 0.5f);
    result.push_back(transform.quaternions[1]/2.0f + 0.5f);
    result.push_back(transform.quaternions[2]/2.0f + 0.5f);
    result.push_back(transform.quaternions[3]/2.0f + 0.5f);

    Vector3D shift = transform.shift - region_centre;

    result.push_back(shift.x/40.0f + 0.5f);
    result.push_back(shift.y/40.0f + 0.5f);
    result.push_back(shift.z/40.0f + 0.5f);

    return result;
}

BestFitFunction::BestFitFunction(std::vector<BestFitFeaturePair> matching_features,
                                 Vector3D region_centre) :
    matching_features(matching_features), region_centre(region_centre) {
    assert(matching_features.size() >= 3);

}

float BestFitFunction::eval(std::vector<float> &params, float iter_frac){
    Transform t = paramsToTransform(params, region_centre);
    float result = 0.0f;

    for(unsigned i = 0; i < matching_features.size(); i++){
        Vector3D p1 = matching_features[i].fpos0;
        Vector3D p2 = matching_features[i].fpos1;

        p1 = applyTransform(t, p1);
        float d = (p1-p2).length();// * matching_features[i].pmatch;
        result += d;
    }

    assert(matching_features.size() > 0);
    result /= (float)matching_features.size();

    return result;
}


struct BestFitWorkerData {
    BestFitWorkerData(std::vector<std::pair<std::vector<float>, float> > &search_results,
                      Util::Semaphore &lock,
                      BestFitFunction *func) :
        search_results(search_results), lock(lock), func(func) {}

    std::vector<std::pair<std::vector<float>, float> > &search_results;
    Util::Semaphore &lock;
    BestFitFunction *func;
};

class BestFitParallelWorker : public ParallelExecutor {
  public:

    BestFitParallelWorker(){};
    ~BestFitParallelWorker(){};

    void performTask(void *task_data, unsigned rank, unsigned size){

        BestFitWorkerData *data = (BestFitWorkerData*)task_data;

        std::vector<float> start_params;
        for(unsigned j = 0; j < 7; j++){
            start_params.push_back((float)rand()/(float)RAND_MAX);
        }

        NelderMead optimiser(start_params);
        std::vector<float> optimised_result = optimiser.optimise(data->func, 7, 400);
        float cur_fitness = data->func->eval(optimised_result, 1.0f);

        data->lock.wait();
        data->search_results.push_back(std::pair<std::vector<float>, float>(optimised_result, cur_fitness));
        data->lock.signal();
    }
};

struct Comp {
    bool operator()(const std::pair<std::vector<float>, float> &a, const std::pair<std::vector<float>, float> &b){
        return a.second < b.second;
    }
};


BestFit::BestFit() {

}

BestFit::~BestFit(){
}

float BestFit::calculateBestFit(const std::vector<BestFitFeaturePair> &matching_features,
                                Transform &result, bool use_prev){

    const unsigned num_iters = 500;
    const float points_fraction = 0.5f;
    std::vector<BestFitFunction*> fitting_functions;

    for(unsigned i = 0; i < num_iters; i++){
        std::vector<BestFitFeaturePair> points_subset;
        unsigned num_features = points_fraction*matching_features.size();
        if(num_features < 3){
            num_features = 3;
        }

        for(unsigned j = 0; j < num_features; j++){
            BestFitFeaturePair p = matching_features[rand()%matching_features.size()];
            points_subset.push_back(p);
        }

        BestFitFunction *new_func = new BestFitFunction(points_subset, hint_shift);
        fitting_functions.push_back(new_func);
    }

    ParallelServer pserver(PSM_PIPELINE);
    BestFitParallelWorker worker_thread;

    std::vector<std::pair<std::vector<float>, float> > search_results;
    Util::Semaphore lock(1);
    Util::Semaphore task_sem;

    std::vector<BestFitWorkerData *> all_data;
    for(unsigned i = 0; i < num_iters; i++){
        BestFitWorkerData *new_data =  new BestFitWorkerData(search_results, lock, fitting_functions[i]);
        ParallelTask task(&worker_thread, new_data, &task_sem);
        pserver.executeParallelTask(task);
        all_data.push_back(new_data);
    }

    for(unsigned i = 0; i < num_iters; i++){ task_sem.wait(); }
    for(unsigned i = 0; i < all_data.size(); i++){ delete all_data[i]; }
    for(unsigned i = 0; i < fitting_functions.size(); i++){ delete fitting_functions[i]; }


    std::vector<float> best_fit_params;
    float best_score = 0.0f;
    for(unsigned i = 0; i < search_results.size(); i++){
        Transform t = paramsToTransform(search_results[i].first, hint_shift);
        float overall_score = getPointsFit(t, matching_features, 0.1f).size()/(float)matching_features.size();
        if(overall_score > best_score){
            best_score = overall_score;
            best_fit_params = search_results[i].first;
        }
    }

    Transform t = paramsToTransform(best_fit_params, hint_shift);
    BestFitFunction *func = new BestFitFunction(getPointsFit(t, matching_features, 0.1f), hint_shift);
    std::vector<float> optimised_result;

    NelderMead optimiser(best_fit_params);
    best_fit_params = optimiser.optimise(func, 7, 1000);

    result = paramsToTransform(best_fit_params, hint_shift);
    float error = func->eval(best_fit_params, 1.0f);
    delete func;

    return error;
}

void BestFit::setHintShift(Vector3D hint){
    hint_shift = hint;
}

std::vector<BestFitFeaturePair> BestFit::getPointsFit(Transform &t, std::vector<BestFitFeaturePair> all_pairs, float max_error){
    std::vector<BestFitFeaturePair> result;

    for(unsigned i = 0; i < all_pairs.size(); i++){
        Vector3D p1 = all_pairs[i].fpos0;
        Vector3D p2 = all_pairs[i].fpos1;

        p1 = applyTransform(t, p1);
        float error = (p1-p2).length();

        if(error < max_error){
            result.push_back(all_pairs[i]);
        }
    }

    return result;
}
