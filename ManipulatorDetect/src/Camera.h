
#ifndef _Camera_H_
#define _Camera_H_

#include <vector>

class Camera {

  public:
    Camera(){};
    virtual ~Camera(){};

    virtual unsigned getCameraImageWidth(void) const = 0;
    virtual unsigned getCameraImageHeight(void) const = 0;

    // Returns the most up to date image from the remote 
    // camera. The format is vector of bytes, 3 bytes per 
    // pixel (RGB?), total vector length is width*height*3.
    virtual void getImage(std::vector<char> &left_buffer, std::vector<char> &right_buffer) = 0;

    virtual bool isActive(void){ return true; }
    virtual unsigned getFrameNum(void) = 0;

};



#endif

