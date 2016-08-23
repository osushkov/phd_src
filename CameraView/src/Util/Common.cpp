
#include "Common.h"
#include "../SIFT/sift.h"

#include <cmath>

#define LEFT_CAMERA_X_OFFSET (-6.0f)
#define RIGHT_CAMERA_X_OFFSET (6.0f)


void Common::extractFeatures(IplImage* img, std::vector<feature> &dump_buffer){

    struct feature* features;
    int n = SIFT::sift_features(img, &features);

    dump_buffer.clear();
    for(int i = 0; i < n; i++){
        dump_buffer.push_back(features[i]);
    }

    free(features);
}

IplImage* Common::imageFromBuffer(const std::vector<unsigned char> &buffer,
                                  unsigned width, unsigned height, int Bpp){
    assert(Bpp == 1 || Bpp == 3);
    IplImage* img = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, Bpp);

    for(unsigned y = 0; y < height; y++){
        for(unsigned x = 0; x < width; x++){
            unsigned index = (x + y*width)*Bpp;
            assert(index < buffer.size());

            if(Bpp == 1){
                setPixel(img, x, y, buffer.at(index));
            }
            else{
                setPixel(img, x, y, buffer.at(index), buffer.at(index+1), buffer.at(index+2));
            }
        }
    }

    return img;
}


void Common::getPixel(IplImage* img, unsigned x, unsigned y, double &val){
    CvScalar s;
    s = cvGet2D(img, y, x);
    val = s.val[0];
}


void Common::getPixel(IplImage* img, unsigned x, unsigned y, double &b, double &g, double &r){
    CvScalar s;
    s = cvGet2D(img, y, x);
    b = s.val[0];
    g = s.val[1];
    r = s.val[2];
}


void Common::setPixel(IplImage* img, unsigned x, unsigned y, double val){
    CvScalar s;
    s.val[0] = val;
    cvSet2D(img, y, x, s);
}


void Common::setPixel(IplImage* img, unsigned x, unsigned y, double b, double g, double r){
    CvScalar s;
    s.val[0] = b;
    s.val[1] = g;
    s.val[2] = r;
    cvSet2D(img, y, x, s);
}

