

#ifndef _Common_H_
#define _Common_H_

#include <cmath>
#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>

#include "Vector3D.h"
#include "Vector2D.h"


namespace Common {


    Vector3D stereoProject(Vector2D left_coord, Vector2D right_coord);

    IplImage* imageFromBuffer(const std::vector<unsigned char> &buffer,
                              unsigned width, unsigned height, int Bpp=3);

    void getPixel(IplImage* img, unsigned x, unsigned y, double &val);
    void getPixel(IplImage* img, unsigned x, unsigned y, double &b, double &g, double &r);

    void setPixel(IplImage* img, unsigned x, unsigned y, double val);
    void setPixel(IplImage* img, unsigned x, unsigned y, double b, double g, double r);

    Vector3D getProjectedCameraPoint(float pix_x, float pix_y);


    Vector3D cameraPointToArmSpace(Vector3D point);
    Vector3D cameraPointToWorldSpace(Vector3D point, float pan, float tilt, float roll);

    Vector3D armPointToWorldSpace(Vector3D point);
    Vector3D worldPointToArmSpace(Vector3D point);

    Vector3D cameraPointToWorldSpace(Vector3D point);

    float curveDistance(const std::vector<Vector3D> &curve1,
                        const std::vector<Vector3D> &curve2);

    inline float normalDistribution(float mean, float sd, float x){
        return exp(-(x-mean)*(x-mean)/(2.0f*sd*sd));
    }

    float average(const std::vector<float> &data);
    float standardDeviation(const std::vector<float> &data);

    Matrix3 quaternionsToMatrix(std::vector<float> quaternions);
    std::vector<float> matrixToQuaternions(Matrix3 &mat);
    void normaliseVector(std::vector<float> &vec);

}

#endif

