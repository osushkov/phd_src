

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

namespace Common {

    bool lineIntersect(Vector3D p1, Vector3D p2, Vector3D p3, Vector3D p4, 
                       Vector3D &pa, Vector3D &pb, float &mua, float &mub);

    bool getClosestFeature(feature f, const std::vector<feature> &all_features, feature &closest_feature);
    void extractFeatures(IplImage* img, std::vector<feature> &dump_buffer);
    Vector3D getProjectedCameraPoint(float pix_x, float pix_y);

}

#endif

