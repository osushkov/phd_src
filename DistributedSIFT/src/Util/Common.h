

#ifndef _Common_H_
#define _Common_H_

#include <cmath>
#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "../SIFT/imgfeatures.h"


namespace Common {

    void extractFeatures(IplImage* img, std::vector<feature> &dump_buffer);
    IplImage* imageFromBuffer(const std::vector<char> &buffer, unsigned width, unsigned height);

}

#endif

