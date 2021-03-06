
#ifndef _RemoteCamera_H_
#define _RemoteCamera_H_

#include "Camera.h"
#include "../Util/ReadWriteLock.h"
#include "../Util/Semaphore.h"

#include <string>
#include <vector>

// The remote camera class provides an abstraction over
// the a camera on a remote computer.
class RemoteCamera : public Camera {
  public:
    RemoteCamera(std::string camera_address, int port);
    ~RemoteCamera();

    unsigned getCameraImageWidth(void) const;
    unsigned getCameraImageHeight(void) const;

    // Returns the most up to date image from the remote
    // camera. The format is vector of bytes, 3 bytes per
    // pixel (RGB?), total vector length is width*height*3.
    bool getImage(std::vector<unsigned char> &lbuffer,
                  std::vector<unsigned char> &rbuffer,
                  bool blocking=true);

    unsigned getFrameNum(void){ return frame_num; }

  private:
    const std::string camera_address;
    const int port;

    unsigned image_width, image_height;
    std::vector<unsigned char> left_buffer, right_buffer;

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

