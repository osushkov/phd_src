
#ifndef _Recorder_H_
#define _Recorder_H_

#include "Camera/Camera.h"
#include "Util/Semaphore.h"
#include <string>
#include <list>

class Recorder {
  public:
    Recorder(std::string dir_path, Camera *rc);
    ~Recorder();

    void startRecording(void);
    void stopRecording(void);
    

  private:
    struct BufferedFrame {
        std::vector<unsigned char> left_buffer, right_buffer;
    };

    std::string dir_path;
    Camera *rc;

    bool am_recording;

    std::list<BufferedFrame*> buffer;
    Util::Semaphore buffer_sem;
    Util::Semaphore finished_writing_sem;


    static void* recorderThread(void *thread_args);
    static void* writerThread(void *thread_args);
};

#endif

