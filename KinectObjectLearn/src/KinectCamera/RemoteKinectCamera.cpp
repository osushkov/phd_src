
#include "RemoteKinectCamera.h"
#include "../Util/ReadWriteLock.h"

#include <iostream>
#include <signal.h>
#include <pthread.h>
#include <cassert>
#include <string>
#include <string.h>

#ifdef _WIN32
#include <winsock.h>
#else
#include <unistd.h>
#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#endif

struct UpdateThreadArgs {
    RemoteKinectCamera *remote_camera;
};


RemoteKinectCamera::RemoteKinectCamera(std::string camera_address, int port) :
        camera_address(camera_address), port(port), 
        rgb_width(0), rgb_height(0), depth_width(0), depth_height(0),
        new_frame(0), new_frame_consumed(0), frame_num(0) {

    if(!cameraConnect()){
        std::cerr << "Error connecting to Remote Kinect Camera" << std::endl;
    }

    getImageSize();

    for(unsigned i = 0; i < rgb_width*rgb_height*3; i++){
        rgb_data.push_back(0);
    }

    for(unsigned i = 0; i < depth_width*depth_height; i++){
        depth_data.push_back(0);
    }

    spawnUpdateThread();
}

RemoteKinectCamera::~RemoteKinectCamera(){
    closesocket(socket_fd);
}

unsigned RemoteKinectCamera::getRGBImageWidth(void) const {
    return rgb_width;
}

unsigned RemoteKinectCamera::getRGBImageHeight(void) const {
    return rgb_height;
}

unsigned RemoteKinectCamera::getDepthImageWidth(void) const {
    return depth_width;
}

unsigned RemoteKinectCamera::getDepthImageHeight(void) const {
    return depth_height;
}

bool RemoteKinectCamera::getImage(std::vector<unsigned char> &out_rgb_buffer,
                                  std::vector<unsigned> &out_depth_buffer,
                                  bool blocking){
    if(blocking){
        new_frame.wait();
    }

    buffer_lock.getReadLock();

    out_rgb_buffer = rgb_data;
    out_depth_buffer = depth_data;

    buffer_lock.releaseReadLock();

    new_frame_consumed.signal();

    return true;
}

bool RemoteKinectCamera::canPlaybackArm(void) const {
    return false;
}

std::vector<float> RemoteKinectCamera::getArmJoints(void) const {
    return std::vector<float>(6, 0.0f);
}

bool RemoteKinectCamera::cameraConnect(void){
    struct sockaddr_in serv_addr;
    struct hostent *he;

    //Connect to Capture.
    if((socket_fd = socket(PF_INET, SOCK_STREAM, 0)) == -1){
        std::cerr << "Could not create socket" << std::endl;
        return false;
    }

    if((he = gethostbyname(camera_address.c_str())) == NULL) {
        std::cerr << "Could not resolve host: " << camera_address << std::endl;
        return false;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    serv_addr.sin_addr = *((struct in_addr*) he->h_addr);
    memset(serv_addr.sin_zero, '\0', sizeof(serv_addr.sin_zero));

    if(connect(socket_fd, (struct sockaddr*) &serv_addr, sizeof(serv_addr)) == -1){
        std::cerr << "Could not connect to remote camera" << std::endl;
        return false;
    }

    return true;
}

bool RemoteKinectCamera::getImageSize(void){
    if(recv(socket_fd, (char*)&rgb_width, sizeof(unsigned), 0) != 4){
        std::cerr << "error receiving image size" << std::endl;
        return false;
    }
    if(recv(socket_fd, (char*)&rgb_height, sizeof(unsigned), 0) != 4){
        std::cerr << "error receiving image size" << std::endl;
        return false;
    }
    if(recv(socket_fd, (char*)&depth_width, sizeof(unsigned), 0) != 4){
        std::cerr << "error receiving image size" << std::endl;
        return false;
    }
    if(recv(socket_fd, (char*)&depth_height, sizeof(unsigned), 0) != 4){
        std::cerr << "error receiving image size" << std::endl;
        return false;
    }

    std::cout << "Received image dimension info. Image dimensions: ("
              << rgb_width << "," << rgb_height << ") (" 
              << depth_width << "," << depth_height << ")" << std::endl;

    return true;
}

void RemoteKinectCamera::spawnUpdateThread(void){
    UpdateThreadArgs *args = new UpdateThreadArgs;
    args->remote_camera = this;

    pthread_t thread_id;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    pthread_create(&thread_id, &attr, &RemoteKinectCamera::update, (void *)(args));
    pthread_attr_destroy(&attr);

}

void* RemoteKinectCamera::update(void *thread_arg){
    UpdateThreadArgs *arg = (UpdateThreadArgs *)thread_arg;
    assert(arg != NULL);
    assert(arg->remote_camera != NULL);

    // Allocate enough space for an image buffer
    int rgb_buffer_size = arg->remote_camera->getRGBImageWidth()*
        arg->remote_camera->getRGBImageHeight()*3;
    int depth_buffer_size = arg->remote_camera->getDepthImageWidth()*
        arg->remote_camera->getDepthImageHeight();
    
    unsigned char *rgb_buffer = new unsigned char[rgb_buffer_size];
    unsigned *depth_buffer = new unsigned[depth_buffer_size];

    int total_received = 0;
    int cur_camera = 0;

    std::vector<unsigned char> tmp_rgb_buffer;
    std::vector<unsigned> tmp_depth_buffer;
    tmp_rgb_buffer.resize(rgb_buffer_size);
    tmp_depth_buffer.resize(depth_buffer_size);
    
    while(true){
        int received;
        int offset = 0;
        int expected_size = rgb_buffer_size*sizeof(unsigned char);

        while(offset < expected_size){
            received = recv(arg->remote_camera->socket_fd, (char*)rgb_buffer + offset, expected_size-offset, 0);
            if(received == SOCKET_ERROR){
                goto end;
            }
            offset += received;
        }

        offset = 0;
        expected_size = depth_buffer_size*sizeof(unsigned);
        while(offset < expected_size){
            received = recv(arg->remote_camera->socket_fd, (char *)depth_buffer + offset, expected_size-offset, 0);
            if(received == SOCKET_ERROR){
                goto end;
            }
            offset += received;
        }

        for(unsigned i = 0; i < rgb_buffer_size; i++){
            tmp_rgb_buffer[i] = rgb_buffer[i];
        }

        for(unsigned i = 0; i < depth_buffer_size; i++){
            tmp_depth_buffer[i] = depth_buffer[i];
        }

        arg->remote_camera->buffer_lock.getWriteLock();

        arg->remote_camera->rgb_data = tmp_rgb_buffer;
        arg->remote_camera->depth_data = tmp_depth_buffer;
        arg->remote_camera->frame_num++;

        arg->remote_camera->buffer_lock.releaseWriteLock();
        arg->remote_camera->new_frame.signal();
        arg->remote_camera->new_frame_consumed.wait(); // wait for the client to take the current frame before proceeding.
    }

end:
    std::cerr << "Error in Remote Kinect Camera connection." << std::endl;
    delete[] rgb_buffer;
    delete[] depth_buffer;

    delete arg;
    pthread_exit((void *)NULL);
    return NULL;
}

