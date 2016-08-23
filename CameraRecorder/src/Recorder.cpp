
#include "Recorder.h"
#include "Util/Timer.h"

#include <unistd.h>
#include <cassert>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <sstream>
#include <fstream>
#include <pthread.h>
#include <iostream>

Recorder::Recorder(std::string dir_path, Camera *rc) :
    dir_path(dir_path), rc(rc) {
    
    assert(rc != NULL);
   
    // Make sure the dir path is a fully formed directory path 
    if(dir_path[dir_path.size()-1] != '/'){
        dir_path = dir_path + "/";
    }

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
        Util::Timer timer;
        timer.start();

        /*while(recorder->rc->getFrameNum() == cur_frame_num){
            usleep(1);
        }*/

        cur_frame_num = recorder->rc->getFrameNum();

        // Save the current frame;
        BufferedFrame *new_frame = new BufferedFrame;
        if(!recorder->rc->getImage(new_frame->left_buffer, new_frame->right_buffer)){
            break;
        }

        recorder->buffer.push_back(new_frame);
        recorder->buffer_sem.signal();
    }

    //recorder->am_recording = false;
    pthread_exit((void *)NULL);
}

void* Recorder::writerThread(void *thread_args){
    Recorder *recorder = (Recorder *)thread_args;
    assert(recorder != NULL);
    
    unsigned cam_width = recorder->rc->getCameraImageWidth();
    unsigned cam_height = recorder->rc->getCameraImageHeight();

    IplImage* img_left = cvCreateImage(cvSize(cam_width, cam_height),IPL_DEPTH_8U,3);
    IplImage* img_right = cvCreateImage(cvSize(cam_width, cam_height),IPL_DEPTH_8U,3);

    std::string record_filename = recorder->dir_path + "record_file.txt";
    std::ofstream record_file(record_filename.c_str());
    record_file << cam_width << " " << cam_height << std::endl;

    unsigned num_written_frames = 0;   
    while(recorder->am_recording || recorder->buffer.size() > 0){
        Util::Timer timer;
        timer.start();

        recorder->buffer_sem.wait();
        std::cout << recorder->buffer.size() << std::endl;

        if(recorder->buffer.size() == 0){
            break;
        }

        std::vector<unsigned char> left_buffer = recorder->buffer.front()->left_buffer;
        std::vector<unsigned char> right_buffer = recorder->buffer.front()->right_buffer;

        delete recorder->buffer.front();
        recorder->buffer.pop_front();

        img_left->imageData = (char*)&(left_buffer[0]);
        img_right->imageData = (char*)&(right_buffer[0]);

        std::stringstream tmp_stream;
        tmp_stream << num_written_frames;
        std::string num_str = tmp_stream.str();

        std::string left_filename = recorder->dir_path + "left_" + num_str + ".png";
        std::string right_filename = recorder->dir_path + "right_" + num_str + ".png";

        cvSaveImage(left_filename.c_str(), img_left);
        cvSaveImage(right_filename.c_str(), img_right);

        timer.stop();
        record_file << "frame: " << num_written_frames << " " << left_filename << " " 
                    << right_filename << " " << timer.getNumElapsedSeconds() << std::endl;

        num_written_frames++;
    }

    cvReleaseImage(&img_left);
    cvReleaseImage(&img_right);

    recorder->finished_writing_sem.signal(); 
    pthread_exit((void *)NULL);
}

