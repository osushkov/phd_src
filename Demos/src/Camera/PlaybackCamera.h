
#ifndef _PlaybackCamere_H_
#define _PlaybackCamera_H_

#include "Camera.h"
#include "../Util/ReadWriteLock.h"
#include "../Util/Semaphore.h"

#include <string>
#include <vector>
#include <fstream>
#include <list>
#include <pthread.h>

// PlaybackCamera is a camera type which allows you to
// use as the image source a previously recorded series of
// images on your harddrive
class PlaybackCamera : public Camera {
  public:
    PlaybackCamera(std::string video_path);
    ~PlaybackCamera();

    unsigned getCameraImageWidth(void) const;
    unsigned getCameraImageHeight(void) const;

    // Returns the most up to date image from the remote
    // camera. The format is vector of bytes, 3 bytes per
    // pixel (RGB?), total vector length is width*height*3.
    bool getImage(std::vector<unsigned char> &left_buffer,
                  std::vector<unsigned char> &right_buffer,
                  bool blocking=true);

    bool canPlaybackArm(void) const { return playback_arm; }
    std::vector<float> getArmJoints(void) const;

    bool isActive(void) const { return is_active; }
    unsigned getFrameNum(void) const { return frame_num; }


  private:
    const std::string video_path;

    unsigned image_width, image_height;
    bool playback_arm;

    std::list< std::vector<unsigned char>* > left_buffers, right_buffers;
    std::list< std::vector<float> > arm_joints_buffers;
    std::vector<float> cur_arm_joints;
    mutable Util::Semaphore buffer_sem;
    mutable Util::Semaphore lock;

    mutable Util::Semaphore end_lock;
    bool should_end;

    pthread_t working_thread;

    std::ifstream video_file;

    bool is_active, finished_files;
    unsigned frame_num;


    bool videoPreLoad(void);

    void spawnUpdateThread(void);
    static void* update(void *thread_args);
};


#endif


