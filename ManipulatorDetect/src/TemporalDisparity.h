
#ifndef _TemporalDisparity_H_
#define _TemporalDisparity_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include "StereoFeatureCorrelation.h"
#include "SIFT/imgfeatures.h"


struct DisparityMatrix {
    float mat[4][4]; // an affine transformation matrix for a 3D point
};

void matrixMultiply(const DisparityMatrix &dmat, float &x, float &y, float &z);

DisparityMatrix computeOptimalDisparityMatrix(const std::vector<StereoFeature> &t1_features, 
                                              const std::vector<StereoFeature> &t2_features,
                                              float &error);

DisparityMatrix computeOptimalDisparityRotationMatrix(const std::vector<StereoFeature> &t1_features, 
                                                      const std::vector<StereoFeature> &t2_features,
                                                      float &error, float &x, float &y, float &z);


void displayDisparity(IplImage* img_t1, IplImage* img_t2, 
                      const std::vector<StereoFeature> &t1_features,
                      const std::vector<StereoFeature> &t2_features);

 
#endif

