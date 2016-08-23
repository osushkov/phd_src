
#include "KinectCamera.h"
#include "RemoteKinectCamera.h"
#include "Recorder.h"
#include <cv.h>
#include <highgui.h>
#include <winsock.h>
#include <iostream>

#define _USE_MATH_DEFINES
#include <math.h>

int main(int argc, char **argv){
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(1,1), &wsaData) != 0) {
        std::cerr << "WSAStartup failed." << std::endl;
        return 0;
    }
    else {
        std::cout << "WSAStartup succeeded" << std::endl;
    }

    KinectCamera *cam = KinectCamera::getLiveCamera(); 
    Recorder recorder("data/kinect_vid.dat", cam);
    recorder.startRecording();
/*
    unsigned rgb_width = cam->getRGBImageWidth();
    unsigned rgb_height = cam->getRGBImageHeight();

    unsigned depth_width = cam->getDepthImageWidth();
    unsigned depth_height = cam->getDepthImageHeight();

    cvNamedWindow("RGB", 1);
    cvNamedWindow("Depth", 1);

    //SIFTGenerator sift_generator(LOCAL_COMPUTE, false, "", cam_width, cam_height); 
    IplImage* img_rgb = cvCreateImage(cvSize(rgb_width, rgb_height), IPL_DEPTH_8U, 3);
    IplImage* img_depth = cvCreateImage(cvSize(depth_width, depth_height), IPL_DEPTH_32F, 1);

    while(cam->isActive()){       
        std::vector<unsigned char> rgb_buffer;
        std::vector<unsigned> depth_buffer;
        
        cam->getImage(rgb_buffer, depth_buffer, false);

        unsigned index = 0;
        for(unsigned y = 0; y < rgb_height; y++){
            for(unsigned x = 0; x < rgb_width; x++){
                for(unsigned i = 0; i < 3; i++){
                    img_rgb->imageData[index] = rgb_buffer[index];
                    index++;
                }
            }
        }
        
        index = 0;
        float* flt_buf = (float*)img_depth->imageData;
        for(unsigned y = 0; y < depth_height; y++){
            for(unsigned x = 0; x < depth_width; x++){
                flt_buf[index] = depth_buffer[index]/2048.0f;
                index++;
            }
        }


        cvShowImage("RGB", img_rgb);
        cvShowImage("Depth", img_depth);

        char key = cvWaitKey(20);
        if(key == 'q'){
            break;
        }
    }
*/
    std::cout << "Stop Recording?" << std::endl;
    getchar();

    recorder.stopRecording();
    return 0;
}