

#ifndef _Common_H_
#define _Common_H_

#include <cmath>
#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include "../Features/SIFT/imgfeatures.h"

#include "Vector3D.h"
#include "Vector2D.h"
#include "Transform.h"

struct StereoFeature;

namespace Common {

    bool isValidValue(float val);
    bool isValidValue(double val);

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

    Vector3D cameraPointToWorldSpace(Vector3D point);

    float curveDistance(const std::vector<Vector3D> &curve1,
                        const std::vector<Vector3D> &curve2);

    inline float normalDistribution(float mean, float sd, float x){
        return exp(-(x-mean)*(x-mean)/(2.0f*sd*sd));
    }

    inline float erf(float z){
        // First several terms of the taylor expansion of the erf function
        float result = 0.0f;
        result = z - powf(z, 3.0f)/3.0f + powf(z, 5.0f)/10.0f - powf(z, 7.0f)/42.0f + powf(z, 9.0f)/216.0f;
        return (2.0f/sqrtf(M_PI)) * result;
    }

    void writeFeature(std::ostream &out, StereoFeature &feature);
    void readFeature(std::istream &in, StereoFeature &feature);

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

