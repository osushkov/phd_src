
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <signal.h>
#include <netdb.h>
#include <cassert>
#include <sstream>

#include "Util/ParallelServer.h"
#include "SIFTGenerator.h"

#define MYPORT 5000


struct WorkUnit {
    std::vector<char> img_buffer_left, img_buffer_right;
    unsigned id;
    
    std::vector<feature> result_left, result_right;
};

std::vector<WorkUnit*> work_units; 
SIFTGenerator sift_generator("data/tmp/", false, 320, 240);


bool receiveWorkUnits(int client_socket_fd){
    int num_work_units = 0;
   
    std::cout << "waiting to received" << std::endl; 
    if(recv(client_socket_fd, (char*)&num_work_units, sizeof(int), 0) < 0){ return false; }
    std::cout << "about to received N units: " << num_work_units << std::endl;

    assert(num_work_units >= 0);    
    std::cout << "nwu: " << num_work_units << std::endl;

    int buffer_size = 0;
    if(recv(client_socket_fd, (char*)&buffer_size, sizeof(int), 0) < 0){ return false; }
    std::cout << "buffer size: " << buffer_size << std::endl;

    for(unsigned i = 0; i < (unsigned)num_work_units; i++){
        WorkUnit *new_work_unit = new WorkUnit;
        new_work_unit->img_buffer_left = std::vector<char>(buffer_size);
        new_work_unit->img_buffer_right = std::vector<char>(buffer_size);
        
        int num_received = 0;
        while(num_received < buffer_size){
            int l = recv(client_socket_fd, (char*)&(new_work_unit->img_buffer_left[num_received]), 
                         buffer_size-num_received, 0);
            if(l < 0){ return false; }
            num_received += l;
        }

        num_received = 0;
        while(num_received < buffer_size){
            int l = recv(client_socket_fd, (char*)&(new_work_unit->img_buffer_right[num_received]), 
                         buffer_size-num_received, 0);
            if(l < 0){ return false; }
            num_received += l;
        }

        int id = 0;
        if(recv(client_socket_fd, (char*)&id, sizeof(int), 0) < 0){ return false; }
        new_work_unit->id = id;

        work_units.push_back(new_work_unit);
    }
    
    return true;
}

bool processWorkUnits(void){

    for(unsigned i = 0; i < work_units.size(); i++){
        sift_generator.putNewFrame(work_units[i]->img_buffer_left, work_units[i]->img_buffer_right, 
                                   work_units[i]->id);
        unsigned id = sift_generator.getNextFrame(work_units[i]->result_left, work_units[i]->result_right);
        assert(id == work_units[i]->id);

    }

    /*
    for(unsigned i = 0; i < work_units.size(); i++){
        unsigned id = sift_generator.getNextFrame(work_units[i].result_left, work_units[i].result_right);
        assert(id == work_units[i].id);
    }
    */

    return true;
}

bool replyWorkUnits(int client_socket_fd){
    for(unsigned i = 0; i < work_units.size(); i++){
        int buffer_size = work_units[i]->result_left.size()*sizeof_feature() + 
                          work_units[i]->result_right.size()*sizeof_feature();

        char *buffer = new char[buffer_size];
        char *orig_buffer = buffer;

        for(unsigned j = 0; j < work_units[i]->result_left.size(); j++){
            write_feature_buffer(buffer, work_units[i]->result_left[j]);
            buffer += sizeof_feature();
        }
        for(unsigned j = 0; j < work_units[i]->result_right.size(); j++){
            write_feature_buffer(buffer, work_units[i]->result_right[j]);
            buffer += sizeof_feature();
        }
        buffer = orig_buffer;


        int num_left_feature = work_units[i]->result_left.size();
        int num_right_feature = work_units[i]->result_right.size();
        int id = work_units[i]->id;

        if(send(client_socket_fd, (char*)&id, sizeof(int), MSG_NOSIGNAL) <= 0 ||      
           send(client_socket_fd, (char*)&num_left_feature, sizeof(int), MSG_NOSIGNAL) <= 0 ||
           send(client_socket_fd, (char*)&num_right_feature, sizeof(int), MSG_NOSIGNAL) <= 0 ||
           send(client_socket_fd, (char*)&buffer_size, sizeof(int), 0) <= MSG_NOSIGNAL){

            delete[] buffer;
            return false; 
        }
 
        int num_sent = 0;
        while(num_sent < buffer_size){
            int l = send(client_socket_fd, buffer+num_sent, buffer_size-num_sent, MSG_NOSIGNAL);
            if(l < 0){ 
                delete[] buffer;
                return false; 
            }

            num_sent += l;
        }

        delete[] buffer;
    }

    return true;
}


void clientHandler(int client_socket_fd){
    while(true){
        if(!receiveWorkUnits(client_socket_fd)){ break; }
        std::cout << "got work units" << std::endl;

        if(!processWorkUnits()){ break; }
        std::cout << "processed work units" << std::endl;

        if(!replyWorkUnits(client_socket_fd)){ break; }
        std::cout << "sent work units" << std::endl;

        for(unsigned i = 0; i < work_units.size(); i++){
            delete work_units[i];
        }
        work_units.clear();
    }

    for(unsigned i = 0; i < work_units.size(); i++){
        delete work_units[i];
    }
    work_units.clear();
}


int main(int argc, char **argv){
    int sockfd, new_fd;  // listen on sock_fd, new connection on new_fd
    struct sockaddr_in my_addr;    // my address information
    struct sockaddr_in their_addr; // connector's address information
    socklen_t sin_size;
    int yes=1;


    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        perror("socket");
        exit(1);
    }

    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1) {
        perror("setsockopt");
        exit(1);
    }
    
    my_addr.sin_family = AF_INET;         // host byte order
    my_addr.sin_port = htons(MYPORT);     // short, network byte order
    my_addr.sin_addr.s_addr = INADDR_ANY; // automatically fill with my IP
    memset(my_addr.sin_zero, '\0', sizeof my_addr.sin_zero);

    if (bind(sockfd, (struct sockaddr *)&my_addr, sizeof my_addr) == -1) {
        perror("bind");
        exit(1);
    }

    if (listen(sockfd, 1) == -1) {
        perror("listen");
        exit(1);
    }


    while(true) {
        std::cout << "Waiting for new connection" << std::endl;
        sin_size = sizeof their_addr;
        if ((new_fd = accept(sockfd, (struct sockaddr *)&their_addr, &sin_size)) == -1) {
            perror("accept");
            continue;
        }
        std::cout << "new connection from: " << inet_ntoa(their_addr.sin_addr) << std::endl;

        clientHandler(new_fd);
        close(new_fd); 
        std::cout << "client disconnected" << std::endl;
    }

    return 0;
}

