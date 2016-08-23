

#ifndef _Common_H_
#define _Common_H_

#include <cmath>
#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "../SIFT/imgfeatures.h"

struct Vector3D {
    float x, y, z;
};

struct StereoFeature;

namespace Common {

    bool lineIntersect(Vector3D p1, Vector3D p2, Vector3D p3, Vector3D p4, 
                       Vector3D &pa, Vector3D &pb, float &mua, float &mub);

    void extractFeatures(IplImage* img, std::vector<feature> &dump_buffer);
    Vector3D getProjectedCameraPoint(float pix_x, float pix_y);
    float featureDist(StereoFeature &feature1, StereoFeature &feature2);
    float vectorDist(Vector3D var1, Vector3D var2);

    Vector3D cameraPointToArmSpace(Vector3D point);

    Vector3D armPointToWorldSpace(Vector3D point);
    Vector3D worldPointToArmSpace(Vector3D point);

    Vector3D cameraPointToWorldSpace(Vector3D point);
}

#endif

