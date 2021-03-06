

#ifndef _Common_H_
#define _Common_H_

#include <cmath>
#include <vector>

#ifdef _WIN32
#include <cv.h>
#include <highgui.h>
#else
#include <opencv/cv.h>
#include <opencv/highgui.h>
#endif

#include <iostream>
#include "../Features/SIFT/imgfeatures.h"

#include "Vector3D.h"
#include "Vector2D.h"
#include "Transform.h"

#define GROUND_HEIGHT 11.5f

struct StereoFeature;

struct ModelFrame {
    std::vector<StereoFeature> features;
    Vector3D view_direction;
};

struct BestFitFeaturePair {
    Vector3D fpos0; // scene
    Vector3D fpos1; // snapshot
    float pmatch;
};

namespace Common {


    Vector3D stereoProject(Vector2D left_coord, Vector2D right_coord);

    void extractFeatures(IplImage* img, std::vector<feature> &dump_buffer);
    IplImage* imageFromBuffer(const std::vector<unsigned char> &buffer,
                              unsigned width, unsigned height, int Bpp=3);

    void getPixel(IplImage* img, unsigned x, unsigned y, double &val);
    void getPixel(IplImage* img, unsigned x, unsigned y, double &b, double &g, double &r);

    void setPixel(IplImage* img, unsigned x, unsigned y, double val);
    void setPixel(IplImage* img, unsigned x, unsigned y, double b, double g, double r);

    Vector3D getProjectedCameraPoint(float pix_x, float pix_y);

    float featureDist(const StereoFeature &feature1, const StereoFeature &feature2);
    float featureDist2(const StereoFeature &feature1, const StereoFeature &feature2);

    std::vector<StereoFeature> findNearestFeatures(const std::vector<StereoFeature> &pool,
                                                    const StereoFeature &target);


    Vector3D cameraPointToArmSpace(Vector3D point);
    Vector3D cameraPointToWorldSpace(Vector3D point, float pan, float tilt, float roll);
    Transform cameraToWorldSpaceTransform(float pan, float tilt, float roll);

    Vector3D armPointToWorldSpace(Vector3D point);
    Vector3D worldPointToArmSpace(Vector3D point);
    Matrix4 armPointToWorldSpaceTransform(void);

    Vector3D cameraPointToWorldSpace(Vector3D point);

    float curveDistance(const std::vector<Vector3D> &curve1,
                        const std::vector<Vector3D> &curve2);

    float uniformNoise(float min, float max);
    float gaussianNoise(float mean, float sd);
    float normalDistribution(float mean, float sd, float x);

    void writeFeature(std::ostream &out, StereoFeature &feature);
    void readFeature(std::istream &in, StereoFeature &feature);

    float average(const std::vector<float> &data);
    float standardDeviation(const std::vector<float> &data);

    
    void normaliseVector(std::vector<float> &vec);
    void formProbabilityDistribution(std::vector<float> &vec);

    IplImage* convertToGray32(const IplImage* img);

    std::vector<Vector3D> basisVectorsFromTriangle(std::vector<Vector3D> &triangle);

    float distributionEntropy(std::vector<float> distribution);
    float KLDivergence(std::vector<float> P, std::vector<float> Q);

    void printVector(std::vector<float> vec);


    void sleep(unsigned ms);
}

#endif

