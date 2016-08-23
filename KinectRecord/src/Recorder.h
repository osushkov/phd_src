
#ifndef _Recorder_H_
#define _Recorder_H_

#include "KinectCamera.h"
#include "Util/Semaphore.h"
#include <string>
#include <list>

class Recorder {
  public:
    Recorder(std::string file_path, KinectCamera *rc);
    ~Recorder();

    void startRecording(void);
    void stopRecording(void);
    

  private:
    struct BufferedFrame {
        std::vector<unsigned char> rgb_buffer;
        std::vector<unsigned> depth_buffer;
    };

    std::string file_path;
    KinectCamera *rc;

    bool am_recording;

    std::list<BufferedFrame*> buffer;
    Util::Semaphore buffer_sem;
    Util::Semaphore finished_writing_sem;

    static void* recorderThread(void *thread_args);
    static void* writerThread(void *thread_args);
};

#endif

