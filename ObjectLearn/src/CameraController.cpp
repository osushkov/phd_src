/*
 * CameraController.cpp
 *
 *  Created on: 01/11/2009
 *      Author: osushkov
 */

#include "CameraController.h"
#include "Features/StereoFeatureCorrelation.h"
#include "Camera/RemoteCamera.h"
#include "Camera/PlaybackCamera.h"
#include "Settings.h"


CameraController::CameraController(bool live_camera, std::string video_path,
                                   bool gen_sift, bool live_sift) :
        live_camera(live_camera), video_path(video_path),
        gen_sift(gen_sift), live_sift(live_sift),
        paused(false), cur_frame(0),
        cam(NULL), img_left(NULL), img_right(NULL),
        sift_generator(NULL) {

    loadCamera();

    if(gen_sift){
        loadSIFTGenerator();
    }
}

CameraController::~CameraController(){
    if(sift_generator != NULL){
        delete sift_generator;
    }

    if(cam != NULL){
        delete cam;
    }

    closeWindow("LeftEye");
    closeWindow("RightEye");
}


bool CameraController::getNewFrame(void){
    // Get the image buffers from the remote camera
    if(!paused){
        if (!cam->getImage(left_cam_buffer, right_cam_buffer)) {
            return false;
        }
    }

    if(img_left != NULL){
        cvReleaseImage(&img_left);
    }
    if(img_right != NULL){
        cvReleaseImage(&img_right);
    }

    img_left = Common::imageFromBuffer(left_cam_buffer, cam_width, cam_height);
    img_right = Common::imageFromBuffer(right_cam_buffer, cam_width, cam_height);

    if (!paused && gen_sift) {
        sift_generator->putNewFrame(left_cam_buffer, right_cam_buffer, cur_frame);
        sift_generator->getNextFrame(left_sift_features, right_sift_features);

        stereo_features = StereoFeatureCorrelation::correlateFeatures(left_sift_features,
                                                                      right_sift_features,
                                                                      cam_height);
    }

    return true;
}

char CameraController::showCameraViewAndContinue(void){
    cvShowImage("LeftEye", img_left);
    cvShowImage("RightEye", img_right);

    char key = cvWaitKey();

    cvReleaseImage(&img_left);
    cvReleaseImage(&img_right);

    return key;
}

void CameraController::createViewWindows(void){
    cvNamedWindow("LeftEye", 1);
    cvMoveWindow("LeftEye", 600, 30);

    cvNamedWindow("RightEye", 1);
    cvMoveWindow("RightEye", 950, 30);
}

void CameraController::closeWindow(std::string name){
    // Just because opencv is being stupid here I put it in a loop
    // to make it work.
    for (int i = 0; i < 2; i++) {
        cvDestroyWindow(name.c_str());
        cvWaitKey(5);
    }
}

std::vector<unsigned char> CameraController::getLeftCamBuffer(void) const {
    return left_cam_buffer;
}

std::vector<unsigned char> CameraController::getRightCamBuffer(void) const {
    return right_cam_buffer;
}

IplImage* CameraController::getLeftImage(void) const {
    return img_left;
}

IplImage* CameraController::getRightImage(void) const {
    return img_right;
}

std::vector<StereoFeature> CameraController::getStereoFeatures(void) const {
    return stereo_features;
}

bool CameraController::haveArmJoints(void) const {
    return cam->canPlaybackArm();
}

std::vector<float> CameraController::getArmJoints(void) const {
    return cam->getArmJoints();
}

void CameraController::setPaused(bool npaused) {
    paused = npaused;
}

bool CameraController::isActive(void) const {
    if(cam == NULL){
        return false;
    }

    return cam->isActive();
}


void CameraController::loadCamera(void){
    if (live_camera) {
        cam = Camera::getLiveCamera();
    }
    else {
        cam = Camera::getPlaybackCamera(video_path);
    }

    cam_width = cam->getCameraImageWidth();
    cam_height = cam->getCameraImageHeight();

    Settings::instance().addSetting("camera", "res_width", (int) cam_width);
    Settings::instance().addSetting("camera", "res_height", (int) cam_height);
}

void CameraController::loadSIFTGenerator(void){
    if (live_sift) {
        sift_generator = new SIFTGenerator(LOCAL_COMPUTE, (video_path.size() > 0),
                                           video_path + "sift_file.dat",
                                           cam_width, cam_height);
    }
    else {
        sift_generator = new SIFTGenerator(LOCAL_FILE_READ, false, video_path + "sift_file.dat",
                                           cam_width, cam_height);
    }
}
