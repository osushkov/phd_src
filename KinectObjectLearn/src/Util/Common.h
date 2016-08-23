

#ifndef _Common_H_
#define _Common_H_

#define _USE_MATH_DEFINES
#include <math.h>

#include <vector>
#include <cv.h>
#include <highgui.h>
#include <iostream>
#include "../Features/SIFT/imgfeatures.h"
#include "../Features/SIFTFeature3D.h"
#include "../KinectCamera/KinectCamera.h"

#include "Vector3D.h"
#include "Vector2D.h"
#include "Transform.h"

namespace Common {

    void sleep(unsigned ms);

    bool isPointInWorkspace(Vector3D wpoint);

    bool isValidValue(float val);
    bool isValidValue(double val);

    void extractFeatures(IplImage* img, std::vector<feature> &dump_buffer);
    void extractFeature(KinectCamera::CorrelatedImage &frame, std::vector<SIFTFeature3D> &out_features);

    IplImage* imageFromCorrelatedFrame(KinectCamera::CorrelatedImage &frame);

    IplImage* imageFromBuffer(const std::vector<unsigned char> &buffer,
                              unsigned width, unsigned height, int Bpp=3);

    void getPixel(IplImage* img, unsigned x, unsigned y, double &val);
    void getPixel(IplImage* img, unsigned x, unsigned y, double &b, double &g, double &r);

    void setPixel(IplImage* img, unsigned x, unsigned y, double val);
    void setPixel(IplImage* img, unsigned x, unsigned y, double b, double g, double r);

    float featureDist(const SIFTFeature3D &feature1, const SIFTFeature3D &feature2);
    float featureDist2(const SIFTFeature3D &feature1, const SIFTFeature3D &feature2);

    std::vector<SIFTFeature3D> findNearestFeatures(const std::vector<SIFTFeature3D> &pool,
                                                   const SIFTFeature3D &target);


    Vector3D cameraPointToWorldSpace(Vector3D point, float pan, float tilt);
    Transform cameraToWorldSpaceTransform(float pan, float tilt);

    Vector3D armPointToWorldSpace(Vector3D point);
    Vector3D worldPointToArmSpace(Vector3D point);

    float curveDistance(const std::vector<Vector3D> &curve1,
                        const std::vector<Vector3D> &curve2);

    float curveDistance2(const std::vector<Vector3D> &curve1,
                         const std::vector<Vector3D> &curve2);

    inline float normalDistribution(float mean, float sd, float x){
        return exp(-(x-mean)*(x-mean)/(2.0f*sd*sd));
    }

    inline float erf(float z){
        // First several terms of the taylor expansion of the erf function
        float result = 0.0f;
        result = z - powf(z, 3.0f)/3.0f + powf(z, 5.0f)/10.0f - powf(z, 7.0f)/42.0f + powf(z, 9.0f)/216.0f;
        return (2.0f/sqrtf((float)M_PI)) * result;
    }

    void writeFeature(std::ostream &out, const SIFTFeature3D &feature);
    void readFeature(std::istream &in, SIFTFeature3D &feature);

    float average(const std::vector<float> &data);
    float standardDeviation(const std::vector<float> &data);
    float maximum(const std::vector<float> &data);

    Matrix3 quaternionsToMatrix(std::vector<float> quaternions);
    std::vector<float> matrixToQuaternions(Matrix3 &mat);
    void normaliseVector(std::vector<float> &vec);

    IplImage* convertToGray32(const IplImage* img);

    std::vector<Vector3D> basisVectorsFromTriangle(std::vector<Vector3D> &triangle);
}

#endif

