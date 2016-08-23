
#include "TemporalDisparity.h"
#include "OptimiserClass/Optimiser.h"
#include "OptimiserClass/NelderMead.h"
#include "OptimiserClass/ParticleSwarm.h"
#include "OptimiserClass/SimulatedAnnealing.h"
#include <cassert>
#include <iostream>
#include <algorithm>

std::vector<StereoFeature> cur_features_t1, cur_features_t2;

/*
std::vector<float> disparityMatrixToVector(const DisparityMatrix &dmat){
    std::vector<float> result;
    result.push_back(dmat.mat[0][0]);
    result.push_back(dmat.mat[0][1]);
    result.push_back(dmat.mat[0][2]);
    result.push_back(dmat.mat[1][0]);
    result.push_back(dmat.mat[1][1]);
    result.push_back(dmat.mat[1][2]);
    return result;
}*/

DisparityMatrix vectorToDisparityMatrix(const std::vector<float> &vec){
    assert(vec.size() == 3);
    DisparityMatrix result;

    result.mat[0][0] = 1.0f;
    result.mat[0][1] = 0.0f;
    result.mat[0][2] = 0.0f;
    result.mat[0][3] = vec[0];

    result.mat[1][0] = 0.0f;
    result.mat[1][1] = 1.0f;
    result.mat[1][2] = 0.0f;
    result.mat[1][3] = vec[1];

    result.mat[2][0] = 0.0f;
    result.mat[2][1] = 0.0f;
    result.mat[2][2] = 1.0f;
    result.mat[2][3] = vec[2];

    result.mat[3][0] = 0.0f;
    result.mat[3][1] = 0.0f;
    result.mat[3][2] = 0.0f;
    result.mat[3][3] = 1.0f;

    return result;
}

void matrixMultiply(const DisparityMatrix &dmat, float &x, float &y, float &z){
    float ox = x, oy = y, oz = z;
    x = dmat.mat[0][0]*ox + dmat.mat[0][1]*oy + dmat.mat[0][2]*oz + dmat.mat[0][3];
    y = dmat.mat[1][0]*ox + dmat.mat[1][1]*oy + dmat.mat[1][2]*oz + dmat.mat[1][3];
    z = dmat.mat[2][0]*ox + dmat.mat[2][1]*oy + dmat.mat[2][2]*oz + dmat.mat[2][3];
}

float computeError(const std::vector<StereoFeature> &t1_features, 
                   const std::vector<StereoFeature> &t2_features,
                   const DisparityMatrix &dmat){

    assert(t1_features.size() == t2_features.size());
    float sum_error = 0.0f;

    for(unsigned i = 0; i < t1_features.size(); i++){
        float tx = t1_features[i].position.x;
        float ty = t1_features[i].position.y;
        float tz = t1_features[i].position.z;

        float x = t2_features[i].position.x;
        float y = t2_features[i].position.y;
        float z = t2_features[i].position.z;

        matrixMultiply(dmat, x, y, z);
        sum_error += sqrtf((x-tx)*(x-tx) + (y-ty)*(y-ty) + (z-tz)*(z-tz));
    }
    if(sum_error != sum_error){ return FLT_MAX; }

    return sum_error;
}

std::vector<float> denormaliseParams(const std::vector<float> &nparams){
    std::vector<float> result(nparams);
    for(unsigned i = 0; i < result.size(); i++){
        result[i] *= 1.0f;
    }
    return result;
}

class DisparityFunction : public OptimisableFunction {
  public:
    float eval(std::vector<float> &params){
        std::vector<float> denormalised_params = denormaliseParams(params);
        DisparityMatrix dmat = vectorToDisparityMatrix(denormalised_params);
        float r = computeError(cur_features_t1, cur_features_t2, dmat);
        return r;
    }
};


DisparityMatrix computeOptimalDisparityMatrix(const std::vector<StereoFeature> &t1_features, 
                                              const std::vector<StereoFeature> &t2_features,
                                              float &error){

    cur_features_t1 = t1_features;
    cur_features_t2 = t2_features;
    assert(cur_features_t1.size() == cur_features_t2.size());

    Optimiser *op = new NelderMead();
    OptimisableFunction *eval_func = new DisparityFunction();

    
    std::vector<float> min_vec = op->optimise(eval_func, 3, 1000, std::vector<float>());
    min_vec = denormaliseParams(min_vec);
    DisparityMatrix dmat = vectorToDisparityMatrix(min_vec);
    error = computeError(t1_features, t2_features, dmat);

    return dmat;
}


//////////////////////////////////////////////////////////////////////////

DisparityMatrix vectorToDisparityRotationMatrix(const std::vector<float> &vec){
    assert(vec.size() == 7);
    DisparityMatrix result;

    float theta = vec[6];
    float c = cosf(theta);
    float s = sinf(theta);
    float t = 1.0f - cosf(theta);
    float x = vec[3];
    float y = vec[4];
    float z = vec[5];

    float length = sqrtf(x*x+y*y+z*z);
    x /= length;
    y /= length;
    z /= length;


    result.mat[0][0] = t*x*x + c;
    result.mat[0][1] = t*x*y - s*z;
    result.mat[0][2] = t*x*z + s*y;
    result.mat[0][3] = 0.0f;

    result.mat[1][0] = t*x*y + s*z;
    result.mat[1][1] = t*y*y + c;
    result.mat[1][2] = t*y*z - s*x;
    result.mat[1][3] = 0.0f;

    result.mat[2][0] = t*x*z - s*y;
    result.mat[2][1] = t*y*z + s*x;
    result.mat[2][2] = t*z*z + c;
    result.mat[2][3] = 0.0f;

    result.mat[3][0] = 0.0f;
    result.mat[3][1] = 0.0f;
    result.mat[3][2] = 0.0f;
    result.mat[3][3] = 1.0f;

    return result;
}

float computeRotationError(const std::vector<StereoFeature> &t1_features, 
                           const std::vector<StereoFeature> &t2_features,
                           const DisparityMatrix &dmat, const float xrp, const float yrp, const float zrp){

    assert(t1_features.size() == t2_features.size());
    float sum_error = 0.0f;

    for(unsigned i = 0; i < t1_features.size(); i++){
        float tx = t1_features[i].position.x;
        float ty = t1_features[i].position.y;
        float tz = t1_features[i].position.z;

        float x = t2_features[i].position.x;
        float y = t2_features[i].position.y;
        float z = t2_features[i].position.z;

        x -= xrp;
        y -= yrp;
        z -= zrp; 
        matrixMultiply(dmat, x, y, z);
        x += xrp;
        y += yrp;
        z += zrp;

        sum_error += sqrtf((x-tx)*(x-tx) + (y-ty)*(y-ty) + (z-tz)*(z-tz));
    }
    if(sum_error != sum_error){ return FLT_MAX; }

    return sum_error;
}

class RotationDisparityFunction : public OptimisableFunction {
  public:
    float eval(std::vector<float> &params){
        std::vector<float> denormalised_params = denormaliseParams(params);
        DisparityMatrix dmat = vectorToDisparityRotationMatrix(denormalised_params);

        float r = computeRotationError(cur_features_t1, cur_features_t2, dmat, 
                                       denormalised_params[0], denormalised_params[1], denormalised_params[2]);
        return r;
    }
};

DisparityMatrix computeOptimalDisparityRotationMatrix(const std::vector<StereoFeature> &t1_features, 
                                                      const std::vector<StereoFeature> &t2_features,
                                                      float &error, float &x, float &y, float &z){
    cur_features_t1 = t1_features;
    cur_features_t2 = t2_features;
    assert(cur_features_t1.size() == cur_features_t2.size());
 
    Optimiser *op = new NelderMead();
    OptimisableFunction *eval_func = new RotationDisparityFunction();

    std::vector<float> min_vec = op->optimise(eval_func, 7, 10000, std::vector<float>());
    min_vec = denormaliseParams(min_vec);
    DisparityMatrix dmat = vectorToDisparityRotationMatrix(min_vec);

    error = computeRotationError(t1_features, t2_features, dmat, min_vec[0], min_vec[1], min_vec[2]);
    x = min_vec[0];
    y = min_vec[1];
    z = min_vec[2];

    return dmat;
}


void displayDisparity(IplImage* img_t1, IplImage* img_t2, 
                      const std::vector<StereoFeature> &t1_features,
                      const std::vector<StereoFeature> &t2_features){

    cvNamedWindow("DisparityDisplay_T1", 1);
    cvNamedWindow("DisparityDisplay_T2", 1);

    
    IplImage *display_t1 = (IplImage*)cvClone(img_t1);
    IplImage *display_t2 = (IplImage*)cvClone(img_t2);

    for(unsigned i = 0; i < t1_features.size(); i++){
        cvCircle(display_t1, cvPoint(t1_features[i].feature_left.x, t1_features[i].feature_left.y), 2, CV_RGB(0, 255, 0), 2);
    }

    for(unsigned i = 0; i < t2_features.size(); i++){
        cvCircle(display_t2, cvPoint(t2_features[i].feature_left.x, t2_features[i].feature_left.y), 2, CV_RGB(0, 255, 0), 2);
    }

    cvShowImage("DisparityDisplay_T1", display_t1);
    cvShowImage("DisparityDisplay_T2", display_t2);
}


