/*
 * SuperQuadricBestFit.cpp
 *
 *  Created on: 28/10/2009
 *      Author: osushkov
 */

#include "SuperQuadricBestFit.h"
#include "../OptimiserClass/Optimiser.h"
#include "../OptimiserClass/RRNelderMead.h"
#include "../OptimiserClass/NelderMead.h"
#include "../OptimiserClass/ParticleSwarm.h"
#include "../OptimiserClass/GeneticAlgorithm.h"
#include "../OptimiserClass/SimulatedAnnealing.h"
#include "../Util/Matrix.h"
#include "../Util/Geometry.h"
#include "../Util/ParallelServer.h"
#include "../Util/Semaphore.h"

#include "../Visualisation/SuperQuadricRenderObject.h"
#include "../Visualisation/SceneRenderer.h"

#include <vector>
#include <algorithm>
#include <limits.h>


static void renderPoints(std::vector<Vector3D> points, Vector3D color){
    PointCloudRenderObject *point_cloud = new PointCloudRenderObject();
    SceneRenderer::instance().addObject(point_cloud);
    point_cloud->setColor(color);

    std::vector<PointCloudPoint> all_points;
    for(unsigned i = 0; i < points.size(); i++){
        PointCloudPoint new_point;
        new_point.pos = points[i];// + Vector3D(0.0f, 0.0f, 10.0f);
        all_points.push_back(new_point);
    }

    point_cloud->addPoints(all_points);
}

static Vector3D applyTransform(const Transform &t, const Vector3D &vec){
    Vector3D result = t.mat*vec - t.shift;
    return result;
}


static SuperQuadricFit paramsToSuperQuadric(const std::vector<float> &params){
    for(unsigned i = 0; i < params.size(); i++){
        assert(params[i] >= 0.0f && params[i] <= 1.0f);
    }

    SuperQuadricFit result;
    result.e1 = params[0]*1.0f + 0.1f;
    result.e2 = params[1]*1.0f + 0.1f;

    result.A = params[2]*4.0f + 0.5f;
    result.B = params[3]*4.0f + 0.5f;
    result.C = params[4]*8.0f + 0.5f;

    result.xt = params[5]+0.1f;
    result.yt = params[5]+0.1f;



    float q0, q1, q2, q3;

    q0 = (params[6]-0.5f)*2.0f;
    q1 = (params[7]-0.5f)*2.0f;
    q2 = (params[8]-0.5f)*2.0f;
    q3 = (params[9]-0.5f)*2.0f;

    float l = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= l;
    q1 /= l;
    q2 /= l;
    q3 /= l;

    result.transform.quaternions.clear();
    result.transform.quaternions.push_back(q0);
    result.transform.quaternions.push_back(q1);
    result.transform.quaternions.push_back(q2);
    result.transform.quaternions.push_back(q3);

    result.transform.mat = Common::quaternionsToMatrix(result.transform.quaternions);

    result.transform.shift.x = (params[10]-0.5f)*6.0f;
    result.transform.shift.y = (params[11]-0.5f)*6.0f;
    result.transform.shift.z = (params[12]-0.5f)*6.0f;

    return result;
}

SuperQuadricBestFitFunction::SuperQuadricBestFitFunction(std::vector<Vector3D> sift_points,
                                                         std::vector<Vector3D> feature_points,
                                                         std::vector<Vector3D> edge_points,
                                                         ObjectShapeVerifier &shape_verifier,
                                                         bool use_silhouette) :
    sift_points(sift_points), feature_points(feature_points), edge_points(edge_points),
    shape_verifier(shape_verifier), use_silhouette(use_silhouette) {
/*
    SceneRenderer::instance().clearObjects();
    renderPoints(sift_points, Vector3D(1.0f, 0.0f, 0.0f));
    renderPoints(feature_points, Vector3D(0.0f, 1.0f, 0.0f));
    renderPoints(edge_points, Vector3D(0.0f, 0.0f, 1.0f));
    getchar();
*/
}

float SuperQuadricBestFitFunction::eval(std::vector<float> &params, float iter_fraction){
    SuperQuadricFit quadric = paramsToSuperQuadric(params);
    SuperQuadric sq(quadric.e1, quadric.e2, quadric.A, quadric.B, quadric.C, quadric.xt, quadric.yt, 0);

    float sift_error = pointsError(quadric, sq, sift_points);
    float feature_error = pointsError(quadric, sq, feature_points);
    float edge_error = pointsError(quadric, sq, edge_points);

    float volume = (quadric.A)*(quadric.B)*0.5f*((quadric.C)*quadric.xt + (quadric.C));
    float background_overlap_penalty = 0.0f;

    if(use_silhouette && false){
        background_overlap_penalty = shape_verifier.calculateModelFeatureOverlapPenalty(sq, quadric.transform);
    }

    assert(Common::isValidValue(sift_error));
    assert(Common::isValidValue(feature_error));
    assert(Common::isValidValue(edge_error));
    return sqrtf(volume) * ((sift_error + feature_error + edge_error)/3.0f + 0.1f*background_overlap_penalty);
}

float SuperQuadricBestFitFunction::pointsError(SuperQuadricFit quadric, SuperQuadric &sq, const std::vector<Vector3D> &points){
    float result = 0.0f;
    if(points.size() == 0){
        return result;
    }

    for(unsigned i = 0; i < points.size(); i++){
        Vector3D tp = applyTransform(quadric.transform, points[i]);
        double error = sq.distance(tp);
        error = error*error;
        assert(Common::isValidValue(error));
        result += error;
    }

    return result/(float)points.size();
}

struct SQBFParallelWorkerData {

    SQBFParallelWorkerData(std::vector<std::pair<std::vector<float>, float> > &search_results,
                           Util::Semaphore &lock, SuperQuadricBestFitFunction *func) :
        search_results(search_results), lock(lock), func(func) {}

    std::vector<std::pair<std::vector<float>, float> > &search_results;
    Util::Semaphore &lock;
    SuperQuadricBestFitFunction *func;
};

static unsigned cur_eval;
class SQBFParallelWorker : public ParallelExecutor {
  public:

    SQBFParallelWorker(){};
    ~SQBFParallelWorker(){};

    void performTask(void *task_data, unsigned rank, unsigned size){

        SQBFParallelWorkerData *data = (SQBFParallelWorkerData*)task_data;

        std::vector<float> start_params;
        for(unsigned j = 0; j < 13; j++){
            start_params.push_back((float)rand()/(float)RAND_MAX);
        }

        NelderMead optimiser(start_params);
        std::vector<float> optimised_result = optimiser.optimise(data->func, 13, 600);
        float cur_fitness = data->func->eval(optimised_result, 1.0f);

        data->lock.wait();
        data->search_results.push_back(std::pair<std::vector<float>, float>(optimised_result, cur_fitness));
        data->lock.signal();

        std::cout << "Finished eval " << cur_eval++ << std::endl;
    }
};

struct Comp {
    bool operator()(const std::pair<std::vector<float>, float> &a, const std::pair<std::vector<float>, float> &b){
        return a.second < b.second;
    }
};


SuperQuadricBestFit::SuperQuadricBestFit(ObjectShapeVerifier &shape_verifier) :
    shape_verifier(shape_verifier){

}

SuperQuadricBestFit::~SuperQuadricBestFit(){

}

std::vector<std::pair<SuperQuadricFit,float> >
SuperQuadricBestFit::calculateShapeHypotheses(std::vector<Vector3D> sift_points,
                                              std::vector<Vector3D> feature_points,
                                              std::vector<Vector3D> edge_points){

    renderPoints(sift_points, Vector3D(1.0f, 0.0f, 0.0f));
    renderPoints(feature_points, Vector3D(0.0f, 1.0f, 0.0f));
    renderPoints(edge_points, Vector3D(0.0f, 0.0f, 1.0f));
    //getchar();

    cur_eval = 0;
    const unsigned num_iters = 1500;
    const float sift_points_fraction = 0.1f;
    const float feature_points_fraction = 0.05f;
    const float edge_points_fraction = 0.05f;

    std::vector<SuperQuadricBestFitFunction*> fitting_functions;
    for(unsigned i = 0; i < num_iters; i++){
        std::vector<Vector3D> sift_points_subset;
        std::vector<Vector3D> feature_points_subset;
        std::vector<Vector3D> edge_points_subset;

        if(sift_points.size() > 0){
            for(unsigned j = 0; j < sift_points_fraction*sift_points.size(); j++){
                Vector3D p = sift_points[rand()%sift_points.size()];
                sift_points_subset.push_back(p);
            }
        }

        if(feature_points.size() > 0){
            for(unsigned j = 0; j < feature_points_fraction*sift_points.size(); j++){
                Vector3D p = feature_points[rand()%feature_points.size()];
                feature_points_subset.push_back(p);
            }
        }

        if(edge_points.size() > 0){
            for(unsigned j = 0; j < edge_points_fraction*edge_points.size(); j++){
                Vector3D p = edge_points[rand()%edge_points.size()];
                edge_points_subset.push_back(p);
            }
        }

        SuperQuadricBestFitFunction *new_func = new SuperQuadricBestFitFunction(sift_points_subset, feature_points_subset, edge_points_subset,
                                                                                shape_verifier, false);
        fitting_functions.push_back(new_func);
    }


    ParallelServer pserver(PSM_PIPELINE);
    SQBFParallelWorker worker_thread;

    std::vector<std::pair<std::vector<float>, float> > search_results;
    Util::Semaphore lock(1);
    Util::Semaphore task_sem;

    std::vector<SQBFParallelWorkerData *> all_data;
    for(unsigned i = 0; i < num_iters; i++){
        SQBFParallelWorkerData *new_data =  new SQBFParallelWorkerData(search_results, lock, fitting_functions[i]);
        ParallelTask task(&worker_thread, new_data, &task_sem);
        pserver.executeParallelTask(task);
        all_data.push_back(new_data);
    }

    for(unsigned i = 0; i < num_iters; i++){ task_sem.wait(); }
    for(unsigned i = 0; i < all_data.size(); i++){ delete all_data[i]; }
    for(unsigned i = 0; i < fitting_functions.size(); i++){ delete fitting_functions[i]; }


    std::vector<float> best_fit_params;
    float best_score = 0.0f;
    float best_fitness = 0.0f;
    for(unsigned i = 0; i < search_results.size(); i++){
        SuperQuadricFit sq_fit = paramsToSuperQuadric(search_results[i].first);

        float sift_points_score = 0.0f, feature_points_score = 0.0f, edge_points_score = 0.0f;

        if(sift_points.size() > 0){
            sift_points_score = getPointsFit(sq_fit, sift_points, 0.2f).size()/(float)sift_points.size();
        }

        if(feature_points.size() > 0){
            feature_points_score = getPointsFit(sq_fit, feature_points, 0.2f).size()/(float)feature_points.size();
        }

        if(edge_points.size() > 0){
            edge_points_score = getPointsFit(sq_fit, edge_points, 0.2f).size()/(float)edge_points.size();
        }

        float overall_score = sift_points_score + 0.25f*feature_points_score + 0.25f*edge_points_score;

        if(overall_score > best_score){
            best_score = overall_score;
            best_fit_params = search_results[i].first;
        }
    }

    for(unsigned i = 0; i < search_results.size(); i++){
        SuperQuadricFit sq_fit = paramsToSuperQuadric(search_results[i].first);
        SuperQuadric sq(sq_fit.e1, sq_fit.e2, sq_fit.A, sq_fit.B, sq_fit.C, sq_fit.xt, sq_fit.yt, 0);
        //float overlap_penalty = shape_verifier.calculateModelFeatureOverlapPenalty(sq, sq_fit.transform);

        float sift_points_score = 0.0f, feature_points_score = 0.0f, edge_points_score = 0.0f;

        if(sift_points.size() > 0){
            sift_points_score = getPointsFit(sq_fit, sift_points, 0.25f).size()/(float)sift_points.size();
        }

        if(feature_points.size() > 0){
            feature_points_score = getPointsFit(sq_fit, feature_points, 0.25f).size()/(float)feature_points.size();
        }

        if(edge_points.size() > 0){
            edge_points_score = getPointsFit(sq_fit, edge_points, 0.25f).size()/(float)edge_points.size();
        }

        float overall_score = sift_points_score + 0.25f*feature_points_score + 0.25f*edge_points_score;
        float overall_fitness = search_results[i].second;// + 0.1f*overlap_penalty;

        if(i == 0 || (overall_score > 0.95f*best_score && overall_fitness < best_fitness)){
            best_fitness = overall_fitness;
            best_fit_params = search_results[i].first;
        }
    }

    std::cout << "best score: " << best_score << std::endl;


    SuperQuadricFit best_fit_sq = paramsToSuperQuadric(best_fit_params);
    std::vector<std::pair<SuperQuadricFit,float> > result;
    //result.push_back(std::pair<SuperQuadricFit,float>(best_fit_sq, 0.0f));

    SuperQuadricBestFitFunction *func = new SuperQuadricBestFitFunction(getPointsFit(best_fit_sq, sift_points, 0.3f),
                                                                        getPointsFit(best_fit_sq, feature_points, 0.3f),
                                                                        getPointsFit(best_fit_sq, edge_points, 0.3f),
                                                                        shape_verifier, true);
    NelderMead optimiser(best_fit_params);
    best_fit_params = optimiser.optimise(func, 13, 500);
    result.push_back(std::pair<SuperQuadricFit,float>(paramsToSuperQuadric(best_fit_params), 0.0f));

    SuperQuadric sq(result[0].first.e1, result[0].first.e2, result[0].first.A, result[0].first.B, result[0].first.C, result[0].first.xt, result[0].first.yt, 0);
    shape_verifier.setDebug(true);
    //std::cout << shape_verifier.calculateModelFeatureOverlapPenalty(sq, result[0].first.transform) << std::endl;

    for(unsigned i = 0; i < std::min<unsigned>(3, result.size()); i++){
        std::cout << result[i].first.e1 << " " << result[i].first.e2 << " "
                  << result[i].first.A << " " << result[i].first.B << " "<< result[i].first.C << " "
                  << result[i].first.xt << " " << result[i].first.yt << std::endl;
        //SuperQuadric sq(result[i].first.e1, result[i].first.e2, result[i].first.A, result[i].first.B, result[i].first.C, result[i].first.xt, result[i].first.yt);
        //shape_verifier.calculateModelSilhouetteScore(sq, result[i].first.transform);
    }


    return result;
}

std::vector<Vector3D> SuperQuadricBestFit::getPointsFit(SuperQuadricFit &sq_fit, std::vector<Vector3D> points, float max_error){
    std::vector<Vector3D> result;

    SuperQuadric sq(sq_fit.e1, sq_fit.e2, sq_fit.A, sq_fit.B, sq_fit.C, sq_fit.xt, sq_fit.yt, 0);
    for(unsigned i = 0; i < points.size(); i++){
        Vector3D tp = applyTransform(sq_fit.transform, points[i]);
        double error = sq.distance(tp);
        if(error < max_error){
            result.push_back(points[i]);
        }
    }

    return result;
}

