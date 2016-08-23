
#ifndef _PlaybackCamere_H_
#define _PlaybackCamera_H_

#include "Camera.h"
#include "Util/ReadWriteLock.h"
#include "Util/Semaphore.h"

#include <string>
#include <vector>
#include <fstream>
#include <list>

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
    void getImage(std::vector<char> &left_buffer, std::vector<char> &right_buffer);

    bool isActive(void){ return is_active; }
    unsigned getFrameNum(void){ return frame_num; }


  private:
    const std::string video_path;

    unsigned image_width, image_height;

    std::list< std::vector<char> > left_buffers, right_buffers;
    Util::Semaphore buffer_sem;
    Util::Semaphore lock;

    std::ifstream video_file;

    bool is_active, finished_files;
    unsigned frame_num;
 
    
    bool videoPreLoad(void);

    void spawnUpdateThread(void);
    static void* update(void *thread_args);
};


#endif

