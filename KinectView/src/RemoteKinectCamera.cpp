
#include "RemoteKinectCamera.h"
#include "ReadWriteLock.h"

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

KinectCamera::CorrelatedImage RemoteKinectCamera::getCorrelatedImage(bool blocking) {
    if(blocking){
        new_frame.wait();
    }

    buffer_lock.getReadLock();

    std::vector<unsigned char> lrgb_buffer = rgb_data;
    std::vector<unsigned> ldepth_buffer = depth_data;

    buffer_lock.releaseReadLock();
    new_frame_consumed.signal();

    std::vector<std::pair<Vector3D,bool> > pixel_positions(rgb_width*rgb_height, std::pair<Vector3D,bool>(Vector3D(0.0f, 0.0f, 0.0f), false));
   
    for(unsigned y = 0; y < depth_height; y++){
        for(unsigned x = 0; x < depth_width; x++){
            float zdepth = KinectCamera::rangeValueToDistance(ldepth_buffer[x + y*depth_width]);
            if(zdepth < 0.1f){
                continue;
            }

            Vector3D world_pos = KinectCamera::depthToWorld(x, y, zdepth);

            Vector2D point_rgb_xy = KinectCamera::worldSpaceToRGBPixelCoord(world_pos);
            if(point_rgb_xy.x < 0.0f || point_rgb_xy.x >= rgb_width ||
               point_rgb_xy.y < 0.0f || point_rgb_xy.y >= rgb_height){
                continue;
            }
            
            unsigned px = (unsigned)point_rgb_xy.x;
            unsigned py = (unsigned)point_rgb_xy.y;
            pixel_positions[px + py*rgb_width] = std::pair<Vector3D,bool>(world_pos, true);
        }
    }
    
    const int x_window = 3;
    const int y_window = 3;
    for(int y = 0; y < rgb_height; y++){
        for(int x = 0; x < rgb_width; x++){
            if(pixel_positions[x + y*rgb_width].second){ continue; }

            int x_ref_left = x;
            int x_ref_right = x;
            for(int i = 1; i < x_window; i++){
                if(x_ref_right == x && 
                   x+i < rgb_width && 
                   pixel_positions[x+i + y*rgb_width].second){
                    
                    x_ref_right = x+i;
                }

                if(x_ref_left == x &&
                   x >= i &&
                   pixel_positions[x-i + y*rgb_width].second){
                    
                    x_ref_left = x-i;
                }
            }

            if(x_ref_left != x && x_ref_right != x){
                Vector3D interp_pos = pixel_positions[x_ref_left + y*rgb_width].first + 
                    (x-x_ref_left)/(x_ref_right-x_ref_left) * 
                    (pixel_positions[x_ref_right + y*rgb_width].first - pixel_positions[x_ref_left + y*rgb_width].first);

                pixel_positions[x + y*rgb_width] = std::pair<Vector3D,bool>(interp_pos, true);
                continue;
            }
            

            int y_ref_up = y;
            int y_ref_down = y;
            for(int i = 1; i < y_window; i++){
                if(y_ref_down == y && 
                   y+i < rgb_height && 
                   pixel_positions[x + (y+i)*rgb_width].second){

                    y_ref_down = y+i;
                }

                if(y_ref_up == y &&
                   y >= i &&
                   pixel_positions[x + (y-i)*rgb_width].second){

                    y_ref_up = y-i;
                }
            }

            if(y_ref_up != y && y_ref_down != y){
                Vector3D interp_pos = pixel_positions[x + y_ref_up*rgb_width].first + 
                    (y-y_ref_up)/(y_ref_down-y_ref_up) * 
                    (pixel_positions[x + y_ref_down*rgb_width].first - pixel_positions[x + y_ref_up*rgb_width].first);

                pixel_positions[x + y*rgb_width] = std::pair<Vector3D,bool>(interp_pos, true);
            }
        }
    }
    

    CorrelatedImage result;
    result.width = rgb_width;
    result.height = rgb_height;

    for(int y = 0; y < rgb_height; y++){
        for(int x = 0; x < rgb_width; x++){
            DepthPixel pixel;
            pixel.color = Vector3D(lrgb_buffer[(x + y*rgb_width)*3 + 2],
                                   lrgb_buffer[(x + y*rgb_width)*3 + 1],
                                   lrgb_buffer[(x + y*rgb_width)*3 + 0]);
            pixel.have_pos = pixel_positions[x + y*rgb_width].second;
            pixel.pos = pixel_positions[x + y*rgb_width].first;
            result.depth_pixels.push_back(pixel);
        }
    }

    return result;
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

