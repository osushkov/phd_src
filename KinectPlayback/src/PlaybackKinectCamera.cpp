
#include "PlaybackKinectCamera.h"
#include "Util/ReadWriteLock.h"

#include <iostream>
#include <pthread.h>
#include <cassert>
#include <string>
#include <Windows.h>

struct UpdateThreadArgs {
    PlaybackKinectCamera *playback_camera;
};

PlaybackKinectCamera::PlaybackKinectCamera(std::string video_path) :
    video_path(video_path), lock(1), end_lock(0), 
    should_end(false), is_active(true), 
    finished_files(false), frame_num(0) {

    if(!videoPreLoad()){
        is_active = false;
    }
    else{
        spawnUpdateThread();
    }
}

PlaybackKinectCamera::~PlaybackKinectCamera(){
    should_end = true;
    end_lock.wait();

    while(rgb_buffers.size() != 0){
        delete rgb_buffers.front();
        rgb_buffers.pop_front();
    }

    while(depth_buffers.size() != 0){
        delete depth_buffers.front();
        depth_buffers.pop_front();
    }
}

unsigned PlaybackKinectCamera::getRGBImageWidth(void) const {
    return rgb_width;
}

unsigned PlaybackKinectCamera::getRGBImageHeight(void) const {
    return rgb_height;
}

unsigned PlaybackKinectCamera::getDepthImageWidth(void) const {
    return depth_width;
}

unsigned PlaybackKinectCamera::getDepthImageHeight(void) const {
    return depth_height;
}

bool PlaybackKinectCamera::getImage(std::vector<unsigned char> &rgb_buffer,
                                    std::vector<unsigned> &depth_buffer,
                                    bool blocking){
    buffer_sem.wait();
    if(finished_files && rgb_buffers.size() == 0){
        is_active = false;
        return false;
    }

    lock.wait();
    rgb_buffer = *(rgb_buffers.front());
    depth_buffer = *(depth_buffers.front());

    delete rgb_buffers.front();
    delete depth_buffers.front();
    rgb_buffers.pop_front();
    depth_buffers.pop_front();

    frame_num++;
    lock.signal();

    if(finished_files && rgb_buffers.size() == 0){
        is_active = false;
        return false;
    }

    return true;
}


bool PlaybackKinectCamera::videoPreLoad(void){
    fptr = fopen(video_path.c_str(), "rb");
    if(fptr == NULL){ return false; }

    fread((void*)&rgb_width, sizeof(unsigned), 1, fptr);
    fread((void*)&rgb_height, sizeof(unsigned), 1, fptr);
    fread((void*)&depth_width, sizeof(unsigned), 1, fptr);
    fread((void*)&depth_height, sizeof(unsigned), 1, fptr);

    return true;
}

void PlaybackKinectCamera::spawnUpdateThread(void){
    pthread_create(&working_thread, NULL, &PlaybackKinectCamera::update, (void *)this);
}

void* PlaybackKinectCamera::update(void *thread_arg){
    assert(thread_arg != NULL);
    PlaybackKinectCamera *playback_camera = (PlaybackKinectCamera *)thread_arg;

    // Allocate enough space for an image buffer
    unsigned rgb_width = playback_camera->getRGBImageWidth();
    unsigned rgb_height = playback_camera->getRGBImageHeight();
    unsigned depth_width = playback_camera->getDepthImageWidth();
    unsigned depth_height = playback_camera->getDepthImageHeight();

    while(!feof(playback_camera->fptr)){
        unsigned frame_number;
        std::vector<unsigned char> *rgb_buffer = new std::vector<unsigned char>(rgb_width*rgb_height*3, 0);
        std::vector<unsigned> *depth_buffer = new std::vector<unsigned>(depth_width*depth_height, 0);

        fread((void*)&frame_number, sizeof(unsigned), 1, playback_camera->fptr);
        fread((void*)&((*rgb_buffer)[0]), sizeof(unsigned char), rgb_width*rgb_height*3, playback_camera->fptr);
        fread((void*)&((*depth_buffer)[0]), sizeof(unsigned), depth_width*depth_height, playback_camera->fptr);

        playback_camera->lock.wait();
        playback_camera->rgb_buffers.push_back(rgb_buffer);
        playback_camera->depth_buffers.push_back(depth_buffer);
        playback_camera->lock.signal();

        playback_camera->buffer_sem.signal();

        while(playback_camera->rgb_buffers.size() > 100){
            while(playback_camera->rgb_buffers.size() > 50){
                Sleep(1);

                if(playback_camera->should_end){
                    goto end;
                }
            }
        }
    }
end:

    fclose(playback_camera->fptr);
    playback_camera->finished_files = true;
    playback_camera->end_lock.signal();
    playback_camera->buffer_sem.signal();
    pthread_exit((void *)NULL);
    return NULL;
}

