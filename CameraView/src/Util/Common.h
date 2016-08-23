

#ifndef _Common_H_
#define _Common_H_

#include <cmath>
#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "../SIFT/imgfeatures.h"


namespace Common {

    void extractFeatures(IplImage* img, std::vector<feature> &dump_buffer);
    IplImage* imageFromBuffer(const std::vector<unsigned char> &buffer,
                              unsigned width, unsigned height, int Bpp=3);

    void getPixel(IplImage* img, unsigned x, unsigned y, double &val);
    void getPixel(IplImage* img, unsigned x, unsigned y, double &r, double &g, double &b);

    void setPixel(IplImage* img, unsigned x, unsigned y, double val);
    void setPixel(IplImage* img, unsigned x, unsigned y, double r, double g, double b);

}

#endif

