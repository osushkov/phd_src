/*
 * CameraController.h
 *
 *  Created on: 01/11/2009
 *      Author: osushkov
 */

#ifndef CAMERACONTROLLER_H_
#define CAMERACONTROLLER_H_

#include "Camera/Camera.h"

#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>


class CameraController {
  public:
    CameraController(bool live_camera, std::string video_path);
    ~CameraController();

    bool getNewFrame(void);

    char showCameraViewAndContinue(void);

    void createViewWindows(void);
    void closeWindow(std::string name);


    std::vector<unsigned char> getLeftCamBuffer(void) const;
    std::vector<unsigned char> getRightCamBuffer(void) const;

    IplImage* getLeftImage(void) const;
    IplImage* getRightImage(void) const;


    bool haveArmJoints(void) const;
    std::vector<float> getArmJoints(void) const;

    void setPaused(bool paused);
    bool isActive(void) const;


  private:
    bool live_camera;
    std::string video_path;

    bool paused;

    unsigned cur_frame;
    Camera *cam;

    unsigned cam_width;
    unsigned cam_height;

    std::vector<unsigned char> left_cam_buffer;
    std::vector<unsigned char> right_cam_buffer;
    IplImage* img_left;
    IplImage* img_right;


    void loadCamera(void);
};



#endif /* CAMERACONTROLLER_H_ */
