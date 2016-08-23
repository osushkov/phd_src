
#ifndef _Camera_H_
#define _Camera_H_

#include <vector>
#include <string>

class Camera {
  public:
    static Camera* getLiveCamera(void);
    static Camera* getPlaybackCamera(std::string path);

    Camera(){};
    virtual ~Camera(){};

    virtual unsigned getCameraImageWidth(void) const = 0;
    virtual unsigned getCameraImageHeight(void) const = 0;

    // Returns the most up to date image from the remote
    // camera. The format is vector of bytes, 3 bytes per
    // pixel (RGB?), total vector length is width*height*3.
    virtual bool getImage(std::vector<unsigned char> &left_buffer,
                          std::vector<unsigned char> &right_buffer,
                          bool block=true) = 0;

    virtual bool canPlaybackArm(void) const = 0;
    virtual std::vector<float> getArmJoints(void) const = 0;

    virtual bool isActive(void) const { return true; }
    virtual unsigned getFrameNum(void) const = 0;

};



#endif

