
#ifndef _Recorder_H_
#define _Recorder_H_

#include "KinectCamera.h"
#include "../Arm/Arm.h"
#include "../Util/Semaphore.h"
#include <string>
#include <list>

class Recorder {
  public:
    Recorder(std::string file_path, KinectCamera *rc);
    Recorder(std::string file_path, KinectCamera *rc, Arm *arm);

    ~Recorder();

    void startRecording(int fps=0);
    void stopRecording(void);
    
  private:
    struct BufferedFrame {
        std::vector<unsigned char> rgb_buffer;
        std::vector<unsigned> depth_buffer;
        std::vector<float> arm_joints;
    };

    std::string file_path;
    KinectCamera *rc;
    Arm *arm;

    bool am_recording;
    int fps;

    std::list<BufferedFrame*> buffer;
    Util::Semaphore buffer_sem;
    Util::Semaphore finished_recording_sem;
    Util::Semaphore finished_writing_sem;

    static void* recorderThread(void *thread_args);
    static void* writerThread(void *thread_args);
};

#endif

