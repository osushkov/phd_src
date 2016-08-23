#include "Recorder.h"
#include "../Util/Timer.h"

#include <cassert>
#include <cstdio>
#include <pthread.h>
#include <iostream>
#include <fstream>

Recorder::Recorder(std::string file_path, KinectCamera *rc) :
        file_path(file_path), rc(rc), arm(NULL) {

    assert(rc != NULL);
}

Recorder::Recorder(std::string file_path, KinectCamera *rc, Arm *arm) :
        file_path(file_path), rc(rc), arm(arm) {

    assert(rc != NULL);
    assert(arm != NULL);

}

Recorder::~Recorder(){
    if(am_recording){
        stopRecording();
    }
}

void Recorder::startRecording(int fps){
    this->fps = fps;
    this->am_recording = true;

    pthread_t thread_id;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    pthread_create(&thread_id, &attr, &Recorder::writerThread, (void *)this);
    pthread_create(&thread_id, &attr, &Recorder::recorderThread, (void *)this);
    pthread_attr_destroy(&attr);
    std::cout << "START RECORDING" << std::endl;
}

void Recorder::stopRecording(void){
    std::cout << "STOP RECORDING" << std::endl;
    am_recording = false;

    if(buffer.size() > 0){ 
        buffer_sem.signal();
    }
    finished_recording_sem.wait();
    finished_writing_sem.wait();
}    

void* Recorder::recorderThread(void *thread_args){
    Recorder *recorder = (Recorder *)thread_args;
    assert(recorder != NULL);

    unsigned cur_frame_num = 0;
    const float fps_delay = (recorder->fps == 0) ? 0.0f : 1.0f/(float)recorder->fps;

    Util::Timer timer;
    timer.start();

    while(recorder->am_recording){
        // Save the current frame;
        BufferedFrame *new_frame = new BufferedFrame();
        recorder->rc->getImage(new_frame->rgb_buffer, new_frame->depth_buffer);
        if(timer.getNumElapsedSecondsSplit() >= fps_delay){
            timer.stop();
            timer.start();

            if(recorder->arm != NULL){
                new_frame->arm_joints = recorder->arm->requestCurrentAngles();
            }

            recorder->buffer.push_back(new_frame);
            recorder->buffer_sem.signal();
        }
        else{
            delete new_frame;
        }
    }

    recorder->finished_recording_sem.signal();
    pthread_exit((void *)NULL);
    return NULL;
}

void* Recorder::writerThread(void *thread_args){
    Recorder *recorder = (Recorder *)thread_args;
    assert(recorder != NULL);

    unsigned rgb_width = recorder->rc->getRGBImageWidth();
    unsigned rgb_height = recorder->rc->getRGBImageHeight();
    unsigned depth_width = recorder->rc->getDepthImageWidth();
    unsigned depth_height = recorder->rc->getDepthImageHeight();

    std::ofstream out_file(recorder->file_path.c_str(), std::ios::out | std::ios::binary);
    
    out_file.write((const char*)&rgb_width, sizeof(unsigned));
    out_file.write((const char*)&rgb_height, sizeof(unsigned));
    out_file.write((const char*)&depth_width, sizeof(unsigned));
    out_file.write((const char*)&depth_height, sizeof(unsigned));

    int have_arm_joints = (int)(recorder->arm != NULL);
    out_file.write((const char*)&have_arm_joints, sizeof(int));

    unsigned num_written_frames = 0;   
    while(recorder->am_recording || recorder->buffer.size() > 0){
        recorder->buffer_sem.wait();

        if(recorder->buffer.size() == 0){
            break;
        }

        std::vector<unsigned char> rgb_buffer = recorder->buffer.front()->rgb_buffer;
        std::vector<unsigned> depth_buffer = recorder->buffer.front()->depth_buffer;
        std::vector<float> arm_joints = recorder->buffer.front()->arm_joints;

        delete recorder->buffer.front();
        recorder->buffer.pop_front();

        out_file.write((const char*)&num_written_frames, sizeof(unsigned));
        out_file.write((const char*)&(rgb_buffer[0]), sizeof(unsigned char)*rgb_width*rgb_height*3);
        out_file.write((const char*)&(depth_buffer[0]), sizeof(unsigned)*depth_width*depth_height);

        if(have_arm_joints){
            out_file.write((const char*)&(arm_joints[0]), sizeof(float)*arm_joints.size());
        }

        num_written_frames++;
    }

    recorder->finished_writing_sem.signal(); 
    pthread_exit((void *)NULL);
    return NULL;
}

