/*
 * CameraController.h
 *
 *  Created on: 01/11/2009
 *      Author: osushkov
 */

#ifndef CAMERACONTROLLER_H_
#define CAMERACONTROLLER_H_

#include "Camera/Camera.h"
#include "Features/StereoFeatureCorrelation.h"
#include "Features/SIFT/sift.h"
#include "Features/SIFT/imgfeatures.h"
#include "Features/SIFTGenerator.h"

#include <vector>


class CameraController {
  public:
    CameraController(bool live_camera, std::string video_path, bool gen_sift, bool live_sift);
    ~CameraController();

    bool getNewFrame(void);

    char showCameraViewAndContinue(void);

    void createViewWindows(void);
    void closeWindow(std::string name);


    std::vector<unsigned char> getLeftCamBuffer(void) const;
    std::vector<unsigned char> getRightCamBuffer(void) const;

    IplImage* getLeftImage(void) const;
    IplImage* getRightImage(void) const;

    std::vector<StereoFeature> getStereoFeatures(void) const;

    bool haveArmJoints(void) const;
    std::vector<float> getArmJoints(void) const;

    void setPaused(bool paused);
    bool isActive(void) const;


  private:
    bool live_camera;
    std::string video_path;
    bool gen_sift, live_sift;

    bool paused;

    unsigned cur_frame;
    Camera *cam;

    unsigned cam_width;
    unsigned cam_height;

    std::vector<unsigned char> left_cam_buffer;
    std::vector<unsigned char> right_cam_buffer;
    IplImage* img_left;
    IplImage* img_right;

    SIFTGenerator *sift_generator;
    std::vector<feature> left_sift_features, right_sift_features;
    std::vector<StereoFeature> stereo_features;


    void loadCamera(void);
    void loadSIFTGenerator(void);

};



#endif /* CAMERACONTROLLER_H_ */
