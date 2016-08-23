/*
 * CameraController.cpp
 *
 *  Created on: 01/11/2009
 *      Author: osushkov
 */

#include "CameraController.h"
#include "Features/SIFTFeature3D.h"
#include "KinectCamera/RemoteKinectCamera.h"
#include "KinectCamera/PlaybackKinectCamera.h"
#include "Settings.h"
#include "Util/Common.h"


CameraController::CameraController(bool gen_sift) :
    live_camera(true), gen_sift(gen_sift),
    live_sift(true), paused(false),
    cur_frame(0), cam(NULL) {

    arm_joints = std::vector<float>(6, 0.0f);
    loadCamera();
}

CameraController::CameraController(std::string video_path, bool gen_sift) :
        live_camera(false), video_path(video_path),
        gen_sift(gen_sift), live_sift(true),
        paused(false), cur_frame(0),
        cam(NULL) { 

    arm_joints = std::vector<float>(6, 0.0f);
    loadCamera();
}

CameraController::CameraController(std::string video_path, std::string sift_path) :
        live_camera(false), video_path(video_path), sift_path(sift_path),
        gen_sift(gen_sift), live_sift(false), paused(false), cur_frame(0),
        cam(NULL) {

    arm_joints = std::vector<float>(6, 0.0f);
    loadCamera();
    preloadSIFTFile();
}

CameraController::~CameraController(){
    if(cam != NULL){
        delete cam;
    }

    closeWindow("RGB");
}


bool CameraController::getNewFrame(void){
    // Get the image buffers from the remote camera
    if(!paused){
        if(!KinectCamera::getCorrelatedImage(cam, correlated_frame)){
            return false;
        }

        if(cam->canPlaybackArm()){
            arm_joints = cam->getArmJoints();
        }
    }

    if (!paused && gen_sift) {
        if(live_sift){
            Common::extractFeature(correlated_frame, features);
        }
        else{
            unsigned num_features;
            sift_file.read((char *)&num_features, sizeof(unsigned));

            features.clear();
            for(unsigned i = 0; i < num_features; i++){
                SIFTFeature3D new_feature;
                read_feature(sift_file, new_feature.sift_feature);
                sift_file.read((char*)&(new_feature.position), sizeof(Vector3D));
                features.push_back(new_feature);
            }
        }
    }

    return true;
}

char CameraController::showImageAndContinue(IplImage *img){
    cvShowImage("RGB", img);
    char key = cvWaitKey(5);

    return key;
}

char CameraController::showCameraViewAndContinue(void){
    IplImage *img = Common::imageFromCorrelatedFrame(correlated_frame);
    cvShowImage("RGB", img);
    char key = cvWaitKey(5);
    
    cvReleaseImage(&img);
    return key;
}

void CameraController::createViewWindows(void){
    cvNamedWindow("RGB", 1);
    cvMoveWindow("RGB", 600, 30);
}

void CameraController::closeWindow(std::string name){
    // Just because opencv is being stupid here I put it in a loop
    // to make it work.
    for (int i = 0; i < 2; i++) {
        cvDestroyWindow(name.c_str());
        cvWaitKey(5);
    }
}

KinectCamera::CorrelatedImage CameraController::getCorrelatedFrame(void) const {
    return correlated_frame;
}

std::vector<SIFTFeature3D> CameraController::getStereoFeatures(void) const {
    return features;
}

bool CameraController::haveArmJoints(void) const {
    return cam->canPlaybackArm();
}

std::vector<float> CameraController::getArmJoints(void) const {
    return arm_joints;
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
        cam = KinectCamera::getLiveCamera();
    }
    else {
        cam = KinectCamera::getPlaybackCamera(video_path);
    }

    unsigned cam_width = cam->getRGBImageWidth();
    unsigned cam_height = cam->getRGBImageHeight();

    Settings::instance().addSetting("camera", "res_width", (int) cam_width);
    Settings::instance().addSetting("camera", "res_height", (int) cam_height);
}

void CameraController::preloadSIFTFile(void){
    sift_file.open(sift_path.c_str(),  std::ios::in | std::ios::binary);
}
