
#include "PlaybackCamera.h"
#include "../Util/ReadWriteLock.h"

#include <iostream>
#include <pthread.h>
#include <cassert>
#include <string>
#include <opencv/cv.h>
#include <opencv/highgui.h>

struct UpdateThreadArgs {
    PlaybackCamera *playback_camera;
};


PlaybackCamera::PlaybackCamera(std::string video_path) :
    video_path(video_path), image_width(0), image_height(0),
    lock(1), end_lock(0), should_end(false),
    is_active(true), finished_files(false), frame_num(0) {

    videoPreLoad();
    spawnUpdateThread();
}

PlaybackCamera::~PlaybackCamera(){
    should_end = true;
    end_lock.wait();

    while(left_buffers.size() != 0){
        delete left_buffers.front();
        left_buffers.pop_front();
    }

    while(right_buffers.size() != 0){
        delete right_buffers.front();
        right_buffers.pop_front();
    }
}

unsigned PlaybackCamera::getCameraImageWidth(void) const {
    return image_width;
}

unsigned PlaybackCamera::getCameraImageHeight(void) const {
    return image_height;
}


bool PlaybackCamera::getImage(std::vector<unsigned char> &left_buffer,
                              std::vector<unsigned char> &right_buffer,
                              bool blocking){
    buffer_sem.wait();
    if(finished_files && left_buffers.size() == 0){
        is_active = false;
        return false;
    }

    lock.wait();
    left_buffer = *(left_buffers.front());
    right_buffer = *(right_buffers.front());

    delete left_buffers.front();
    delete right_buffers.front();
    left_buffers.pop_front();
    right_buffers.pop_front();

    frame_num++;
    lock.signal();

    if(finished_files && left_buffers.size() == 0){
        is_active = false;
        return false;
    }

    return true;
}


bool PlaybackCamera::videoPreLoad(void){
    std::string full_path = video_path + "record_file.txt";
    video_file.open(full_path.c_str());

    if(!video_file.good()){ return false; }

    video_file >> image_width;
    video_file >> image_height;

    return true;
}

void PlaybackCamera::spawnUpdateThread(void){
    pthread_create(&working_thread, NULL, &PlaybackCamera::update, (void *)this);
}

void* PlaybackCamera::update(void *thread_arg){
    assert(thread_arg != NULL);
    PlaybackCamera *playback_camera = (PlaybackCamera *)thread_arg;

    // Allocate enough space for an image buffer
    unsigned width = playback_camera->getCameraImageWidth();
    unsigned height = playback_camera->getCameraImageHeight();
    unsigned buffer_size = width*height*3;

    IplImage *img_left, *img_right;

    std::string tmp, left_frame_filename, right_frame_filename;
    unsigned frame_num = 0;
    float wait_time;

    std::vector<unsigned char> left_buffer(width*height*3, 0), right_buffer(width*height*3, 0);

    while(!playback_camera->video_file.eof()){

        playback_camera->video_file >> tmp; // read the "frame:" bit
        playback_camera->video_file >> frame_num;

        playback_camera->video_file >> left_frame_filename;
        playback_camera->video_file >> right_frame_filename;

        playback_camera->video_file >> wait_time;

        img_left = cvLoadImage(left_frame_filename.c_str());
        img_right = cvLoadImage(right_frame_filename.c_str());

        assert((unsigned)img_left->width == width && (unsigned)img_left->height == height);
        assert((unsigned)img_right->width == width && (unsigned)img_right->height == height);

        // Wait for the time till the next frame.
        /*while(wait_time > 1.0f){
            sleep(1);
            wait_time -= 1.0f;
        }*/
        //usleep(wait_time*1000000*2);

        std::vector<unsigned char> *left_buffer = new std::vector<unsigned char>(width*height*3, 0);
        std::vector<unsigned char> *right_buffer = new std::vector<unsigned char>(width*height*3, 0);

        for(unsigned i = 0; i < buffer_size; i++){
            left_buffer->at(i) = (unsigned char)img_left->imageData[i];
            right_buffer->at(i) = (unsigned char)img_right->imageData[i];
        }

        playback_camera->lock.wait();
        playback_camera->left_buffers.push_back(left_buffer);
        playback_camera->right_buffers.push_back(right_buffer);
        playback_camera->lock.signal();

        playback_camera->buffer_sem.signal();

        cvReleaseImage(&img_left);
        cvReleaseImage(&img_right);

        while(playback_camera->left_buffers.size() > 100){
            while(playback_camera->left_buffers.size() > 50){
                sleep(1);

                if(playback_camera->should_end){
                    goto end;
                }
            }
        }
    }
end:

    playback_camera->finished_files = true;
    playback_camera->end_lock.signal();
    playback_camera->buffer_sem.signal();
    pthread_exit((void *)NULL);
}

