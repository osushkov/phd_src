
#include "Common.h"
#include "../SIFT/sift.h"

#include <cmath>

void Common::extractFeatures(IplImage* img, std::vector<feature> &dump_buffer){
    
    struct feature* features;
    int n = SIFT::sift_features(img, &features);

    dump_buffer.clear();
    for(int i = 0; i < n; i++){
        dump_buffer.push_back(features[i]);
    }

    free(features); 
}

IplImage* Common::imageFromBuffer(const std::vector<char> &buffer, unsigned width, unsigned height){
    IplImage* img = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);

    for(unsigned i = 0; i < buffer.size(); i++){
        img->imageData[i] = buffer[i];
    }

    return img;
}

