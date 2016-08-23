
#ifndef _RemoteKinectCamera_H_
#define _RemoteKinectCamera_H_

#include "KinectCamera.h"
#include "Util/ReadWriteLock.h"
#include "Util/Semaphore.h"

#include <string>
#include <vector>

// The remote camera class provides an abstraction over
// the a camera on a remote computer.
class RemoteKinectCamera : public KinectCamera {
  public:
    RemoteKinectCamera(std::string camera_address, int port);
    ~RemoteKinectCamera();

    unsigned getRGBImageWidth(void) const;
    unsigned getRGBImageHeight(void) const;

    unsigned getDepthImageWidth(void) const;
    unsigned getDepthImageHeight(void) const;

    CorrelatedImage getCorrelatedImage(bool blocking=true);

    // Returns the most up to date image from the remote
    // camera. The format is vector of bytes, 3 bytes per
    // pixel (RGB?), total vector length is width*height*3.
    bool getImage(std::vector<unsigned char> &out_rgb_buffer,
                  std::vector<unsigned> &out_depth_buffer,
                  bool blocking=true);

    unsigned getFrameNum(void){ return frame_num; }

  private:
    const std::string camera_address;
    const int port;

    unsigned rgb_width, rgb_height;
    unsigned depth_width, depth_height;

    std::vector<unsigned char> rgb_data;
    std::vector<unsigned> depth_data;

    Util::ReadWriteLock buffer_lock;
    Util::Semaphore new_frame, new_frame_consumed;

    int socket_fd; // socket id for talking to the remote computer/camera
    unsigned frame_num;

    bool cameraConnect(void);
    bool getImageSize(void);

    void spawnUpdateThread(void);
    static void* update(void *thread_args);
};

#endif

