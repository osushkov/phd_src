
#include "PlaybackKinectCamera.h"
#include "../Util/ReadWriteLock.h"

#include <iostream>
#include <pthread.h>
#include <cassert>
#include <string>
#include <Windows.h>
#include <fstream>

struct UpdateThreadArgs {
    PlaybackKinectCamera *playback_camera;
};

PlaybackKinectCamera::PlaybackKinectCamera(std::string video_path) :
        video_path(video_path), lock(1), end_lock(0), 
        should_end(false), is_active(true), 
        finished_files(false), frame_num(0) {

    last_arm_joints = std::vector<float>(6, 0.0f);
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

    while(arm_joints_buffers.size() != 0){
        delete arm_joints_buffers.front();
        arm_joints_buffers.pop_front();
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

    if(have_arm_joints){
        last_arm_joints = *(arm_joints_buffers.front());
        delete arm_joints_buffers.front();
        arm_joints_buffers.pop_front();
    }

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

bool PlaybackKinectCamera::canPlaybackArm(void) const {
    return (bool)have_arm_joints;
}

std::vector<float> PlaybackKinectCamera::getArmJoints(void) const {
    return last_arm_joints;
}

bool PlaybackKinectCamera::videoPreLoad(void){
    in_file.open(video_path.c_str(), std::ios::in | std::ios::binary);

    in_file.read((char*)&rgb_width, sizeof(unsigned));
    in_file.read((char*)&rgb_height, sizeof(unsigned));
    in_file.read((char*)&depth_width, sizeof(unsigned));
    in_file.read((char*)&depth_height, sizeof(unsigned));
    in_file.read((char*)&have_arm_joints, sizeof(int));
    
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

    while(!playback_camera->in_file.eof()){
        unsigned frame_number;
        std::vector<unsigned char> *rgb_buffer = new std::vector<unsigned char>(rgb_width*rgb_height*3, 0);
        std::vector<unsigned> *depth_buffer = new std::vector<unsigned>(depth_width*depth_height, 0);
        std::vector<float> *arm_joints_buffer = NULL;

        playback_camera->in_file.read((char*)&frame_number, sizeof(unsigned));
        playback_camera->in_file.read((char*)&((*rgb_buffer)[0]), sizeof(unsigned char)*rgb_width*rgb_height*3);
        playback_camera->in_file.read((char*)&((*depth_buffer)[0]), sizeof(unsigned)*depth_width*depth_height);

        if(playback_camera->have_arm_joints){
            arm_joints_buffer = new std::vector<float>(6, 0.0f);
            playback_camera->in_file.read((char*)&((*arm_joints_buffer)[0]), sizeof(float)*6);
        }

        playback_camera->lock.wait();
        playback_camera->rgb_buffers.push_back(rgb_buffer);
        playback_camera->depth_buffers.push_back(depth_buffer);

        if(playback_camera->have_arm_joints){
            playback_camera->arm_joints_buffers.push_back(arm_joints_buffer);
        }

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

    playback_camera->finished_files = true;
    playback_camera->end_lock.signal();
    playback_camera->buffer_sem.signal();
    pthread_exit((void *)NULL);
    return NULL;
}

