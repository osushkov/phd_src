
#include <iostream>
#include <cstdio>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <sstream>

#include "Camera/Camera.h"
#include "Recorder.h"


int main(int argc, char **argv){
    Camera *camera = Camera::getLiveCamera(); //getPlaybackCamera("data/arm_bulb/");
     
    unsigned cam_width = camera->getCameraImageWidth();
    unsigned cam_height = camera->getCameraImageHeight();

    IplImage* img_left = cvCreateImage(cvSize(cam_width, cam_height),IPL_DEPTH_8U,3);
    IplImage* img_right = cvCreateImage(cvSize(cam_width, cam_height),IPL_DEPTH_8U,3);

    cvNamedWindow("Left Eye", 1);
    cvNamedWindow("Right Eye", 1);

    std::vector<unsigned char> left_cam_buffer, right_cam_buffer;
    left_cam_buffer.resize(cam_width*cam_height*3);
    right_cam_buffer.resize(cam_width*cam_height*3);

    Recorder recorder("data/tmp/", camera);
    recorder.startRecording();

    while(true){
        char key = cvWaitKey(2);		//Need this to update the image windows.
        if(key == 'q'){
            break;
        }

        // Get the image buffers from the remote camera
        if(!camera->getImage(left_cam_buffer, right_cam_buffer)){
            break;
        }

        img_left->imageData = (char*)&(left_cam_buffer[0]);
        img_right->imageData = (char*)&(right_cam_buffer[0]);

        cvShowImage("Left Eye", img_left);
        cvShowImage("Right Eye", img_right);
    }
    
    recorder.stopRecording();
    delete camera;

    return 0;
}

