
#include "RemoteCamera.h"
#include "Util/ReadWriteLock.h"

#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <cassert>
#include <string>


struct UpdateThreadArgs {
    RemoteCamera *remote_camera;

};


RemoteCamera::RemoteCamera(std::string camera_address, int port) : 
    camera_address(camera_address), port(port), image_width(0), image_height(0) {

    cameraConnect();
    getImageSize();

    for(unsigned i = 0; i < image_width*image_height*3; i++){
        left_buffer.push_back(0);
        right_buffer.push_back(0);
    }

    spawnUpdateThread();
}

RemoteCamera::~RemoteCamera(){
    close(socket_fd);
}

unsigned RemoteCamera::getCameraImageWidth(void) const {
    return image_width;
}

unsigned RemoteCamera::getCameraImageHeight(void) const {
    return image_height;
}


void RemoteCamera::getLeftImage(std::vector<char> &buffer){
    buffer_lock.getReadLock();
    buffer = left_buffer; 
    buffer_lock.releaseReadLock();
}

void RemoteCamera::getRightImage(std::vector<char> &buffer){
    buffer_lock.getReadLock();
    buffer = right_buffer;
    buffer_lock.releaseReadLock();
}


bool RemoteCamera::cameraConnect(void){
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

bool RemoteCamera::getImageSize(void){
    char tmp_buffer[64] = {'\0'};

    if(recv(socket_fd, tmp_buffer, 4, 0) != 4){
        std::cerr << "Could not receive full image height buffer" << std::endl;
    }
	image_height = (unsigned)atoi(tmp_buffer);
    if(image_height == 0 || image_height > 10000){ // if image height is 0 or implausibly large, report problem.
        std::cerr << "Problem with image height: " << image_height << std::endl;
    }
	
    if(recv(socket_fd, tmp_buffer, 5, 0) != 5){
        std::cerr << "Could not receive full image width buffer" << std::endl;
    }
	image_width = atoi(tmp_buffer);
    if(image_width == 0 || image_width > 10000){ // if image width is 0 or implausibly large, report problem.
        std::cerr << "Problem with image width: " << image_width << std::endl;
    }

    std::cout << "Received image dimension info. Image dimensions: ("
              << image_width << "," << image_height << ")" << std::endl;

    return true;
}

void RemoteCamera::spawnUpdateThread(void){
    UpdateThreadArgs *args = new UpdateThreadArgs;
    args->remote_camera = this;

    pthread_t thread_id;
    pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    pthread_create(&thread_id, &attr, &RemoteCamera::update, (void *)(args));
    pthread_attr_destroy(&attr);
}

void* RemoteCamera::update(void *thread_arg){
    UpdateThreadArgs *arg = (UpdateThreadArgs *)thread_arg;
    assert(arg != NULL);
    assert(arg->remote_camera != NULL);

    // Allocate enough space for an image buffer
    unsigned buffer_size = arg->remote_camera->getCameraImageWidth()*
                           arg->remote_camera->getCameraImageHeight()*3;
    char *img_buffer = new char[buffer_size];
    int total_received = 0;
    int cur_camera = 0;

    std::vector<char> tmp_left_buffer, tmp_right_buffer;
    tmp_left_buffer.resize(buffer_size);
    tmp_right_buffer.resize(buffer_size);


    int bytes_received = recv(arg->remote_camera->socket_fd, img_buffer, buffer_size, 0);
    while(bytes_received > 0){
        total_received += bytes_received;

        if(total_received == (int)buffer_size) {
            
            if(cur_camera == 0){
                for(unsigned i = 0; i < buffer_size; i++){
                    tmp_left_buffer[i] = img_buffer[i];
                }
            }
            else{
                for(unsigned i = 0; i < buffer_size; i++){
                    tmp_right_buffer[i] = img_buffer[i];
                }
            }

            if(cur_camera == 1){
                arg->remote_camera->buffer_lock.getWriteLock();

                assert(arg->remote_camera->left_buffer.size() == buffer_size);
                assert(arg->remote_camera->right_buffer.size() == buffer_size);
                arg->remote_camera->left_buffer = tmp_left_buffer;
                arg->remote_camera->right_buffer = tmp_right_buffer;

                arg->remote_camera->buffer_lock.releaseWriteLock();
            }

            cur_camera = (cur_camera+1)%2; // switch which camera we will be receiving from
            total_received = 0;
            bytes_received = recv(arg->remote_camera->socket_fd, img_buffer, buffer_size, 0);
        }
        else{
            bytes_received = 
                recv(arg->remote_camera->socket_fd, img_buffer+total_received, buffer_size-total_received, 0);
        }
    }

    delete[] img_buffer;
    delete arg;
    pthread_exit((void *)NULL);
}

