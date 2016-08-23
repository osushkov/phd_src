
#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <winsock.h>
#include <pthread.h>
#include <libfreenect.h>
#include "Semaphore.h"

#define SERVER_NAME "localhost"
#define SERVER_PORT 9999

const unsigned RGB_WIDTH = 640;
const unsigned RGB_HEIGHT = 480;

const unsigned DEPTH_WIDTH = 640;
const unsigned DEPTH_HEIGHT = 480;

unsigned char *rgb_buf_front = new unsigned char[RGB_WIDTH*RGB_HEIGHT*3];
unsigned char *rgb_buf_back = new unsigned char[RGB_WIDTH*RGB_HEIGHT*3];

unsigned *depth_buf_front = new unsigned[DEPTH_WIDTH*DEPTH_HEIGHT];
unsigned *depth_buf_back = new unsigned[DEPTH_WIDTH*DEPTH_HEIGHT];

Util::Semaphore global_lock(1);
Util::Semaphore rgb_sem(0);
Util::Semaphore depth_sem(0);

int m_socket;
freenect_context *f_ctx;
freenect_device *f_dev;
pthread_t freenect_thread_id;

bool die = false;
Util::Semaphore *kinect_thread_sem;

bool initialiseSockets(void){
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(1,1), &wsaData) != 0) {
        std::cerr << "WSAStartup failed." << std::endl;
        return false;
    }
    else {
        std::cout << "WSAStartup succeeded" << std::endl;
    }

    struct hostent *he;
    he = gethostbyname(SERVER_NAME);
    if(he == NULL){
        perror("gethostbyname");
        return false;
    }

    m_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (m_socket == INVALID_SOCKET) {
        std::cerr << "Error at socket(): " << WSAGetLastError() << std::endl;
        WSACleanup();
        return false;
    }

    // Bind
    sockaddr_in service;
    service.sin_family = AF_INET;
    service.sin_addr = *((struct in_addr*) he->h_addr_list[0]);
    service.sin_port = htons(SERVER_PORT);
    if(bind(m_socket, (SOCKADDR*)&service, sizeof(service)) == SOCKET_ERROR) {
        std::cerr << "bind() failed: " << WSAGetLastError() << std::endl;
        closesocket(m_socket);
        return false;
    }

    // Listen 
    if (listen( m_socket, 1) == SOCKET_ERROR) {
        printf("listen(): Error listening on socket %ld.\n", WSAGetLastError());
        return false;
    }

    return true;
}

void *freenect_thread(void *arg){
    while(!die && freenect_process_events(f_ctx) >= 0 );
    kinect_thread_sem->signal();
    return NULL;
}

void rgb_cb(freenect_device *dev, void *rgb, uint32_t timestamp){
    for(unsigned i = 0; i < RGB_WIDTH*RGB_HEIGHT; i++){
        rgb_buf_back[i*3 + 0] = ((unsigned char*)rgb)[i*3 + 2];
        rgb_buf_back[i*3 + 1] = ((unsigned char*)rgb)[i*3 + 1];
        rgb_buf_back[i*3 + 2] = ((unsigned char*)rgb)[i*3 + 0];
    }

    global_lock.wait();

    unsigned char *tmp_buf = rgb_buf_back;
    rgb_buf_back = rgb_buf_front;
    rgb_buf_front = tmp_buf;

    global_lock.signal();

    if(rgb_sem.getCount() == 0){
        rgb_sem.signal();
    }
}

void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp){
    uint16_t *depth = (uint16_t*)v_depth;

    for(unsigned i = 0; i < DEPTH_WIDTH*DEPTH_HEIGHT; i++){
        depth_buf_back[i] = depth[i];
    }

    global_lock.wait();

    unsigned *tmp_buf = depth_buf_back;
    depth_buf_back = depth_buf_front;
    depth_buf_front = tmp_buf;

    global_lock.signal();

    if(depth_sem.getCount() == 0){
        depth_sem.signal();
    }
}

bool kinectInit(void){
    if (freenect_init(&f_ctx, NULL) < 0) {
        std::cerr << "freenect_init() failed" << std::endl;
        return false;
    }

    freenect_set_log_level(f_ctx, FREENECT_LOG_ERROR);
    freenect_select_subdevices(f_ctx, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));

    if (freenect_open_device(f_ctx, &f_dev, 0) < 0) {
        std::cerr << "Could not open device" << std::endl;
        return false;
    }

    freenect_set_tilt_degs(f_dev, 0);
    freenect_set_led(f_dev, LED_RED);
    freenect_set_depth_callback(f_dev, depth_cb);
    freenect_set_video_callback(f_dev, rgb_cb);
    freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB));
    freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));
    freenect_start_depth(f_dev);
    freenect_start_video(f_dev);

    die = false;
    kinect_thread_sem = new Util::Semaphore(0);
    pthread_create(&freenect_thread_id, NULL, freenect_thread, NULL);
    return true;
}

void kinectClose(void){
    die = true;
    kinect_thread_sem->wait();
    delete kinect_thread_sem;

    freenect_stop_depth(f_dev);
    freenect_stop_video(f_dev);   

    freenect_close_device(f_dev);
    freenect_shutdown(f_ctx);
}

int main(int argc, char **argv){
    if(!initialiseSockets()){
        std::cerr << "Error initialising sockets, exiting..." << std::endl;
        return 1;
    }

    while(true){
        int client_socket;

        std::cout << "Waiting for Connection..." << std::endl;
        struct sockaddr_in their_addr;
        int sin_size;

        if ((client_socket = accept(m_socket, (struct sockaddr *)&their_addr, &sin_size)) == -1) {
            std::cerr << "Error accepting connection" << std::endl;
            continue;
        }
        std::cout << "Received new Connection" << std::endl;

        if(!kinectInit()){
            std::cerr << "Error initialising Kinect, exiting..." << std::endl;
            return 1;
        }

        if(send(client_socket, (char *)&RGB_WIDTH, sizeof(unsigned int), 0) == SOCKET_ERROR){
            goto end;
        }
        if(send(client_socket, (char *)&RGB_HEIGHT, sizeof(unsigned int), 0) == SOCKET_ERROR){
            goto end;
        }
        if(send(client_socket, (char *)&DEPTH_WIDTH, sizeof(unsigned int), 0) == SOCKET_ERROR){
            goto end;
        }
        if(send(client_socket, (char *)&DEPTH_HEIGHT, sizeof(unsigned int), 0) == SOCKET_ERROR){
            goto end;
        }

        unsigned char *local_rgb_buf = new unsigned char[RGB_WIDTH*RGB_HEIGHT*3];
        unsigned *local_depth_buf = new unsigned[DEPTH_WIDTH*DEPTH_HEIGHT];

        while(true){
            rgb_sem.wait();
            depth_sem.wait();
            global_lock.wait();
            
            for(unsigned y = 0; y < RGB_HEIGHT; y++){
                for(unsigned x = 0; x < RGB_WIDTH; x++){
                    unsigned index0 = x + y*RGB_WIDTH;
                    unsigned index1 = (RGB_WIDTH-x-1) + (RGB_HEIGHT-y-1)*RGB_WIDTH;
                    local_rgb_buf[index0*3 + 0] = rgb_buf_front[index1*3 + 0];
                    local_rgb_buf[index0*3 + 1] = rgb_buf_front[index1*3 + 1];
                    local_rgb_buf[index0*3 + 2] = rgb_buf_front[index1*3 + 2];
                }
            }

            for(unsigned y = 0; y < DEPTH_HEIGHT; y++){
                for(unsigned x = 0; x < DEPTH_WIDTH; x++){
                    unsigned index0 = x + y*DEPTH_WIDTH;
                    unsigned index1 = (DEPTH_WIDTH-x-1) + (DEPTH_HEIGHT-y-1)*DEPTH_WIDTH;
                    local_depth_buf[index0] = depth_buf_front[index1];
                }
            }
            //memcpy(local_rgb_buf, rgb_buf_front, RGB_WIDTH*RGB_HEIGHT*3*sizeof(unsigned char));
            //memcpy(local_depth_buf, depth_buf_front, DEPTH_WIDTH*DEPTH_HEIGHT*sizeof(unsigned));

            global_lock.signal();

            int size = RGB_WIDTH*RGB_HEIGHT*3*sizeof(unsigned char);
            if(send(client_socket, (char *)local_rgb_buf, size, 0) == SOCKET_ERROR){
                goto end;
            }

            size = DEPTH_WIDTH*DEPTH_HEIGHT*sizeof(unsigned);
            if(send(client_socket, (char *)local_depth_buf, size, 0) == SOCKET_ERROR){
                goto end;
            }
        }

end:
        std::cout << "Connection Closed" << std::endl << std::endl;
        closesocket(client_socket);
        kinectClose();
    }

    return 0;
}