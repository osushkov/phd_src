
#ifndef _PlaybackKinectCamera_H_
#define _PlaybackKinectCamera_H_

#include "KinectCamera.h"
#include "Util/ReadWriteLock.h"
#include "Util/Semaphore.h"

#include <pthread.h>
#include <string>
#include <vector>

// The playback camera class provides an abstraction over
// the a Kinect video stored on the disk.
class PlaybackKinectCamera : public KinectCamera {
  public:
    PlaybackKinectCamera(std::string video_path);
    ~PlaybackKinectCamera();

    unsigned getRGBImageWidth(void) const;
    unsigned getRGBImageHeight(void) const;

    unsigned getDepthImageWidth(void) const;
    unsigned getDepthImageHeight(void) const;

    // Returns the most up to date image from the remote
    // camera. The format is vector of bytes, 3 bytes per
    // pixel (RGB?), total vector length is width*height*3.
    bool getImage(std::vector<unsigned char> &out_rgb_buffer,
                  std::vector<unsigned> &out_depth_buffer,
                  bool blocking=true);

    unsigned getFrameNum(void){ return frame_num; }

  private:
    const std::string video_path;

    unsigned rgb_width, rgb_height;
    unsigned depth_width, depth_height;

    std::list< std::vector<unsigned char>* > rgb_buffers;
    std::list< std::vector<unsigned>* > depth_buffers;

    mutable Util::Semaphore buffer_sem;
    mutable Util::Semaphore lock;

    mutable Util::Semaphore end_lock;
    bool should_end;

    pthread_t working_thread;

    FILE *fptr;

    bool is_active, finished_files;
    unsigned frame_num;


    bool videoPreLoad(void);

    void spawnUpdateThread(void);
    static void* update(void *thread_args);
};

#endif

