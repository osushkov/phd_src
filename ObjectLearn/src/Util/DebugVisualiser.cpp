/*
 * DebugVisualiser.cpp
 *
 *  Created on: 22/10/2009
 *      Author: osushkov
 */

#include "DebugVisualiser.h"
#include "Common.h"

#include <cassert>


DebugVisualiser::DebugVisualiser(){

}

DebugVisualiser::~DebugVisualiser(){

}

void DebugVisualiser::renderInfo(std::vector<std::pair<Vector2D,Vector2D> > matched_features,
                                 IplImage *imgA, IplImage *imgB){

    assert(imgA != NULL && imgB != NULL);
    assert(imgA->width == imgB->width && imgA->height == imgB->height);

    cvNamedWindow("Debug", 1);
    IplImage *img = cvCreateImage(cvSize(imgA->width*2, imgA->height), IPL_DEPTH_8U, 3);

    for(int y = 0; y < imgA->height; y++){
        for(int x = 0; x < imgA->width; x++){
            double r, g, b;
            Common::getPixel(imgA, x, y, b, g, r);
            Common::setPixel(img, x, y, b, g, r);

            Common::getPixel(imgB, x, y, b, g, r);
            Common::setPixel(img, x+imgA->width, y, b, g, r);
        }
    }

    for(unsigned i = 0; i < matched_features.size(); i++){
        CvPoint pos0 = cvPoint(matched_features[i].first.x, matched_features[i].first.y);
        CvPoint pos1 = cvPoint(matched_features[i].second.x + imgA->width,
                               matched_features[i].second.y);
        cvLine(img, pos0, pos1, CV_RGB(255, 0, 0), 1, 8, 0);
    }

    cvShowImage("Debug", img);
    cvWaitKey();
}
