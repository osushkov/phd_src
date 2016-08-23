
#include <iostream>
#include <cstdio>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <sstream>

#include "Util/Common.h"
#include "Camera/Camera.h"
#include "SIFTGenerator.h"


int main(int argc, char **argv){
    Camera *cam = Camera::getLiveCamera(); 
 
    unsigned cam_width = cam->getCameraImageWidth();
    unsigned cam_height = cam->getCameraImageHeight();

    cvNamedWindow("Left", 1);
    cvNamedWindow("Right", 1);

    std::vector<unsigned char> left_cam_buffer, right_cam_buffer;
    left_cam_buffer.resize(cam_width*cam_height*3);
    right_cam_buffer.resize(cam_width*cam_height*3);

    //SIFTGenerator sift_generator(LOCAL_COMPUTE, false, "", cam_width, cam_height); 

    while(cam->isActive()){
        if(!cam->getImage(left_cam_buffer, right_cam_buffer)){ break; }
        unsigned frame_num = cam->getFrameNum();

        /*
        sift_generator.putNewFrame(left_cam_buffer, right_cam_buffer, frame_num);

        std::vector<feature> left_features, right_features;
        sift_generator.getNextFrame(left_features, right_features);
        */

        IplImage* img_left = Common::imageFromBuffer(left_cam_buffer, cam_width, cam_height);
        IplImage* img_right = Common::imageFromBuffer(right_cam_buffer, cam_width, cam_height);

        /*
        for(unsigned j = 0; j < left_features.size(); j++){
            draw_features(img_left, &(left_features[j]), 1);
        }
        for(unsigned j = 0; j < right_features.size(); j++){
            draw_features(img_right, &(right_features[j]), 1);
        }*/

        cvShowImage("Left", img_left);
        cvShowImage("Right", img_right);

        char key = cvWaitKey(2);		//Need this to update the image windows.
        if(key == 'q'){
            break;
        }

        cvReleaseImage(&img_left);
        cvReleaseImage(&img_right);
    }

    return 0;
}

