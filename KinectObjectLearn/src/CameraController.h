/*
 * CameraController.h
 *
 *  Created on: 01/11/2009
 *      Author: osushkov
 */

#ifndef CAMERACONTROLLER_H_
#define CAMERACONTROLLER_H_

#include "KinectCamera/KinectCamera.h"
#include "Features/SIFTFeature3D.h"
#include "Features/SIFT/sift.h"
#include "Features/SIFT/imgfeatures.h"

#include <vector>
#include <fstream>


class CameraController {
  public:
    CameraController(bool gen_sift);
    CameraController(std::string video_path, bool gen_sift);
    CameraController(std::string video_path, std::string sift_path);
    ~CameraController();

    bool getNewFrame(void);

    char showImageAndContinue(IplImage *img);
    char showCameraViewAndContinue(void);

    void createViewWindows(void);
    void closeWindow(std::string name);

    KinectCamera::CorrelatedImage getCorrelatedFrame(void) const;
    std::vector<SIFTFeature3D> getStereoFeatures(void) const;

    bool haveArmJoints(void) const;
    std::vector<float> getArmJoints(void) const;

    void setPaused(bool paused);
    bool isActive(void) const;


  private:
    const bool live_camera;
    const std::string video_path, sift_path;
    const bool gen_sift, live_sift;

    bool paused;

    unsigned cur_frame;
    KinectCamera *cam;
    std::ifstream sift_file;
    
    KinectCamera::CorrelatedImage correlated_frame;
    std::vector<SIFTFeature3D> features;
    std::vector<float> arm_joints;

    void loadCamera(void);
    void preloadSIFTFile(void);
};

#endif /* CAMERACONTROLLER_H_ */
