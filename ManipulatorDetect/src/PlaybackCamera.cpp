
#include "PlaybackCamera.h"
#include "Util/ReadWriteLock.h"

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
    lock(1),
    is_active(true), finished_files(false), frame_num(0) {

    videoPreLoad(); 
    spawnUpdateThread();
}

PlaybackCamera::~PlaybackCamera(){
}

unsigned PlaybackCamera::getCameraImageWidth(void) const {
    return image_width;
}

unsigned PlaybackCamera::getCameraImageHeight(void) const {
    return image_height;
}


void PlaybackCamera::getImage(std::vector<char> &left_buffer, std::vector<char> &right_buffer){
    buffer_sem.wait();

    lock.wait();
    left_buffer = left_buffers.front(); 
    right_buffer = right_buffers.front();

    left_buffers.pop_front();
    right_buffers.pop_front();
    frame_num++;
    lock.signal();

    if(finished_files && left_buffers.size() == 0){
        is_active = false;
    }
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
    UpdateThreadArgs *args = new UpdateThreadArgs;
    args->playback_camera = this;

    pthread_t thread_id;
    pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    pthread_create(&thread_id, &attr, &PlaybackCamera::update, (void *)(args));
    pthread_attr_destroy(&attr);
}

void* PlaybackCamera::update(void *thread_arg){
    UpdateThreadArgs *arg = (UpdateThreadArgs *)thread_arg;
    assert(arg != NULL);
    assert(arg->playback_camera != NULL);

    // Allocate enough space for an image buffer
    unsigned width = arg->playback_camera->getCameraImageWidth();
    unsigned height = arg->playback_camera->getCameraImageHeight();
    unsigned buffer_size = width*height*3;

    IplImage *img_left, *img_right;


    std::string tmp, left_frame_filename, right_frame_filename;
    unsigned frame_num = 0;
    float wait_time;

    std::vector<char> left_buffer(width*height*3, 0), right_buffer(width*height*3, 0);

    while(!arg->playback_camera->video_file.eof()){
    
        arg->playback_camera->video_file >> tmp; // read the "frame:" bit
        arg->playback_camera->video_file >> frame_num;

        arg->playback_camera->video_file >> left_frame_filename;      
        arg->playback_camera->video_file >> right_frame_filename;

        arg->playback_camera->video_file >> wait_time;

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

        for(unsigned i = 0; i < buffer_size; i++){
            left_buffer[i] = img_left->imageData[i];
            right_buffer[i] = img_right->imageData[i];
        }

        arg->playback_camera->lock.wait();
        arg->playback_camera->left_buffers.push_back(left_buffer);
        arg->playback_camera->right_buffers.push_back(right_buffer);
        arg->playback_camera->lock.signal();

        arg->playback_camera->buffer_sem.signal();
        
        cvReleaseImage(&img_left);
        cvReleaseImage(&img_right);

        while(arg->playback_camera->left_buffers.size() > 500){
            while(arg->playback_camera->left_buffers.size() > 200){
                sleep(1);
            }
        }
    }

    arg->playback_camera->finished_files = true;
    delete arg;
    pthread_exit((void *)NULL);
}

