
#include "KinectCamera.h"
#include "RemoteKinectCamera.h"
#include "PlaybackKinectCamera.h"
#include "Visualisation/SceneRenderer.h"
#include "Visualisation/PointCloudRenderObject.h"
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

    SceneRenderer::instance().initialise();
    PointCloudRenderObject *point_cloud = new PointCloudRenderObject();
    SceneRenderer::instance().addObject(point_cloud);

    KinectCamera *cam = KinectCamera::getPlaybackCamera("data/kinect_vid.dat"); 
 
    unsigned rgb_width = cam->getRGBImageWidth();
    unsigned rgb_height = cam->getRGBImageHeight();
    unsigned depth_width = cam->getDepthImageWidth();
    unsigned depth_height = cam->getDepthImageHeight();

    cvNamedWindow("RGB", 1);
    cvNamedWindow("RGBD", 1);

    //SIFTGenerator sift_generator(LOCAL_COMPUTE, false, "", cam_width, cam_height); 
    IplImage* img_rgb = cvCreateImage(cvSize(rgb_width, rgb_height), IPL_DEPTH_8U, 3);
    IplImage* img_rgbd = cvCreateImage(cvSize(rgb_width, rgb_height), IPL_DEPTH_8U, 3);

    while(cam->isActive()){
        KinectCamera::CorrelatedImage correlated_image = KinectCamera::getCorrelatedImage(cam);

        std::vector<PointCloudPoint> new_points;
        unsigned added = 0;

        cvSet(img_rgbd, cvScalar(0,0,0));
        for(unsigned i = 0; i < correlated_image.depth_pixels.size(); i++){
            if(correlated_image.depth_pixels[i].have_pos){
                PointCloudPoint point;
                
                point.color = correlated_image.depth_pixels[i].color;
                point.color.scale(1.0f/255.0f);
                
                point.pos = correlated_image.depth_pixels[i].pos;
                point.pos.scale(10.0f);  
                point.pos.x *= -1;
                point.pos.y *= -1;
                
                if(point.pos.length() < 10.0f){
                    point.pos.z -= 6.0f;
                    new_points.push_back(point);
                }
                added++;

                img_rgbd->imageData[i*3 + 0] = correlated_image.depth_pixels[i].color.z;
                img_rgbd->imageData[i*3 + 1] = correlated_image.depth_pixels[i].color.y;
                img_rgbd->imageData[i*3 + 2] = correlated_image.depth_pixels[i].color.x;
            }

            img_rgb->imageData[i*3 + 0] = correlated_image.depth_pixels[i].color.z;
            img_rgb->imageData[i*3 + 1] = correlated_image.depth_pixels[i].color.y;
            img_rgb->imageData[i*3 + 2] = correlated_image.depth_pixels[i].color.x;
        }

        point_cloud->updatePoints(new_points);

        cvShowImage("RGB", img_rgb);
        cvShowImage("RGBD", img_rgbd);

        char key = cvWaitKey(10);		//Need this to update the image windows.
        if(key == 'q'){
            break;
        }
    }

    getchar();
    return 0;
}