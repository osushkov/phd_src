
#ifndef _KinectCamera_H_
#define _KinectCamera_H_

#include <vector>
#include <string>
#include "Util/Vector3D.h"
#include "Util/Vector2D.h"

class KinectCamera {
  public:
    struct DepthPixel {
        Vector3D color;
        Vector3D pos;
        bool have_pos;
    };

    struct CorrelatedImage {
        unsigned width, height;
        std::vector<DepthPixel> depth_pixels;
    };

    static KinectCamera* getLiveCamera(void);
    static KinectCamera* getPlaybackCamera(std::string path);

    KinectCamera(){};
    virtual ~KinectCamera(){};

    virtual unsigned getRGBImageWidth(void) const = 0;
    virtual unsigned getRGBImageHeight(void) const = 0;

    virtual unsigned getDepthImageWidth(void) const = 0;
    virtual unsigned getDepthImageHeight(void) const = 0;

    virtual CorrelatedImage getCorrelatedImage(bool blocking=true) = 0;

    // Returns the most up to date image from the remote
    // camera. The format is vector of bytes, 3 bytes per
    // pixel (RGB?), total vector length is width*height*3.
    virtual bool getImage(std::vector<unsigned char> &rgb_buffer,
                          std::vector<unsigned> &depth_buffer,
                          bool block=true) = 0;

    virtual bool isActive(void){ return true; }
    virtual unsigned getFrameNum(void) = 0;

    static Vector3D depthToWorld(unsigned x, unsigned y, float depth_m);
    static Vector2D worldSpaceToRGBPixelCoord(Vector3D coord);
    static float rangeValueToDistance(unsigned range_val);
   

};

#endif
