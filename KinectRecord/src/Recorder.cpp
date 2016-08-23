#include "Recorder.h"
#include "Util/Timer.h"

#include <cassert>
#include <cstdio>
#include <pthread.h>
#include <iostream>

Recorder::Recorder(std::string file_path, KinectCamera *rc) :
        file_path(file_path), rc(rc) {

    assert(rc != NULL);
}

Recorder::~Recorder(){
    stopRecording();
}

void Recorder::startRecording(void){
    pthread_t thread_id;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    pthread_create(&thread_id, &attr, &Recorder::writerThread, (void *)this);
    pthread_create(&thread_id, &attr, &Recorder::recorderThread, (void *)this);
    pthread_attr_destroy(&attr);
}

void Recorder::stopRecording(void){
    std::cout << "STOP RECORDING" << std::endl;
    am_recording = false;

    if(buffer.size() > 0){ 
        buffer_sem.signal();
        finished_writing_sem.wait();
    }
}    

void* Recorder::recorderThread(void *thread_args){
    Recorder *recorder = (Recorder *)thread_args;
    assert(recorder != NULL);

    unsigned cur_frame_num = 0;
    while(recorder->am_recording){
        // Save the current frame;
        BufferedFrame *new_frame = new BufferedFrame;
        recorder->rc->getImage(new_frame->rgb_buffer, new_frame->depth_buffer);
        recorder->buffer.push_back(new_frame);
        recorder->buffer_sem.signal();
    }

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

    FILE *fptr = fopen(recorder->file_path.c_str(), "wb");

    fwrite(&rgb_width, sizeof(unsigned), 1, fptr);
    fwrite(&rgb_height, sizeof(unsigned), 1, fptr);
    fwrite(&depth_width, sizeof(unsigned), 1, fptr);
    fwrite(&depth_height, sizeof(unsigned), 1, fptr);

    unsigned num_written_frames = 0;   
    while(recorder->am_recording || recorder->buffer.size() > 0){
        recorder->buffer_sem.wait();
        std::cout << recorder->buffer.size() << std::endl;

        if(recorder->buffer.size() == 0){
            break;
        }

        std::vector<unsigned char> rgb_buffer = recorder->buffer.front()->rgb_buffer;
        std::vector<unsigned> depth_buffer = recorder->buffer.front()->depth_buffer;

        delete recorder->buffer.front();
        recorder->buffer.pop_front();

        fwrite((void*)&num_written_frames, sizeof(unsigned), 1, fptr);
        fwrite((void*)&(rgb_buffer[0]), sizeof(unsigned char), rgb_width*rgb_height*3, fptr);
        fwrite((void*)&(depth_buffer[0]), sizeof(unsigned), depth_width*depth_height, fptr);

        num_written_frames++;
    }

    fclose(fptr);
    recorder->finished_writing_sem.signal(); 
    pthread_exit((void *)NULL);
    return NULL;
}

