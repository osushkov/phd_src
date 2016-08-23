#include <iostream>
#include <cassert>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <signal.h>
#include <netdb.h>

#include "ArmProtocol.h"
#include "DXL/DXL.h"


#define REMOTE_ARM_ADDRESS "humanoid-leftarm"
#define REMOTE_ARM_PORT 5004

#define NECK_PORT "/dev/ttyUSB1"
#define NECK_SPEED 63

#define HAND_PORT "/dev/ttyUSB2"
#define HAND_SPEED 150

#define MYPORT 5006    // the port users will be connecting to

#define BARRET_HAND_ADDRESS "humanoid-rightbrain.ai.cse.unsw.edu.au"
#define BARRET_HAND_PORT 9998

unsigned hand_release_position[] = { 850, 500, 485, 524 };
unsigned hand_open_position[] = { 850, 170, 485, 850 };
unsigned hand_closed_position[] = { 620, 390, 695, 650 };

// 90deg left, middle, 90deg right 
unsigned neck_pan_map[] = { 120, 430, 740 };

// straight up, straigh ahead, straight down
unsigned neck_tilt_map[] = { 665, 365, 65 };

// 90deg left, horizontal, 90deg right
unsigned neck_roll_map[] = { 140, 450, 740 };


struct timeval last_command_time;
int arm_socket_fd;
int bhand_socket_fd;

DXLChain arm_chain;
DXL *arm_motors[4];


void initialiseNeck(void){
    DXLChain chain;
	DXL *motors[3];

    chain.open(NECK_PORT);
	for (int i = 0; i < 3; i++){
		motors[i] = chain.getDXL(i);
    }

    // 65 degrees looking down.
    motors[0]->moveToPosn(neck_pan_map[1], NECK_SPEED);
    motors[1]->moveToPosn(148, NECK_SPEED);
    motors[2]->moveToPosn(neck_roll_map[1], NECK_SPEED);

	std::cout << "Initialised Neck" << std::endl;
}

void getHandMotorLoads(int *loads){
    for(unsigned i = 0; i < 4; i++){
        int cur_load = 0;
        arm_motors[i]->getLoad(&cur_load);
        loads[i] = cur_load;
    }
}

int getMaxHandMotorLoad(void){
    int loads[4];
    getHandMotorLoads(loads);

    int max = 0;
    for(unsigned i = 0; i < 4; i++){
        if(loads[i] > max){
            max = loads[i];
        }
    }
    return max;
}

void closeHand(float amount, int speed){
    if(amount < 0.0f){ amount = 0.0f; }
    if(amount > 1.0f){ amount = 1.0f; }

    for(unsigned i = 0; i < 4; i++){
        arm_motors[i]->moveToPosn(amount*hand_closed_position[i] + (1.0f-amount)*hand_open_position[i], speed);
    }    
}

void softCloseHand(float start_amount){
    while(start_amount < 1.0f){
        closeHand(start_amount, HAND_SPEED);
        usleep(1000);

        int max_load = getMaxHandMotorLoad();
        if(max_load > 375){
            break;
        }

        if(max_load > 200){
            start_amount += 0.05;
        }
        else{
            start_amount += 0.15;
        }
    }
}

void openHand(float amount, int speed){
    if(amount < 0.0f){ amount = 0.0f; }
    if(amount > 1.0f){ amount = 1.0f; }
    
    for(unsigned i = 0; i < 4; i++){
        arm_motors[i]->moveToPosn(amount*hand_open_position[i] + (1.0f-amount)*hand_closed_position[i], speed);
    }
}

void releaseHand(int speed){
    for(unsigned i = 0; i < 4; i++){
        arm_motors[i]->moveToPosn(hand_release_position[i], speed);
    }
}

void initialiseHand(void){
    arm_chain.open(HAND_PORT);
	for (int i = 0; i < 4; i++){
		arm_motors[i] = arm_chain.getDXL(i);
    }

	//open hand
	openHand(0.8f, HAND_SPEED);
	std::cout << "Initialised Hand" << std::endl;
}

bool connectToArm(void){
    struct sockaddr_in serv_addr;
    struct hostent *he;

    // Set the last arm command time
    gettimeofday(&last_command_time, NULL);


    if((arm_socket_fd = socket(PF_INET, SOCK_STREAM, 0)) == -1){
        return false;
    }

    if((he = gethostbyname(REMOTE_ARM_ADDRESS)) == NULL) {
        return false;
	}

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(REMOTE_ARM_PORT);
	serv_addr.sin_addr = *((struct in_addr*) he->h_addr);
	memset(serv_addr.sin_zero, '\0', sizeof(serv_addr.sin_zero));
	if((connect(arm_socket_fd, (struct sockaddr*) &serv_addr, sizeof(serv_addr))) == -1) {
		std::cerr << "Could not connect to arm at: " << REMOTE_ARM_ADDRESS << ":" 
                  << REMOTE_ARM_PORT << std::endl;
		return false;
	}
    

	char buffer[32] = {0};
	int bytes_received = recv(arm_socket_fd, buffer, 32, 0);

	if(bytes_received < 1) {
	    std::cerr << "Could not receive init buffer from: "	 << REMOTE_ARM_ADDRESS << ":" 
                  << REMOTE_ARM_PORT << std::endl;
        return false;
	}
    std::cout << "Connected to arm" << std::endl;

    return true;
}

void sendBytes(char *buf, unsigned num, int socket_fd){
    assert(buf != NULL);

    int bytes_sent = 0;
    while(bytes_sent < (int)num){
        int size = send(socket_fd, buf+bytes_sent, num-bytes_sent, 0);
        if(size <= 0){ break; }
        bytes_sent += size;
    }
    assert(bytes_sent == (int)num);
}

void sendFloats(float *fbuf, unsigned num, int socket_fd){
    assert(fbuf != NULL);

    char buffer[1024];
    for(unsigned i = 0; i < num; i++){
        sprintf(buffer, "%f\n\r", fbuf[i]);
        sendBytes(buffer, strlen(buffer), socket_fd);
    }
}

void sendRawFloats(float *fbuf, unsigned num, int socket_fd){
    assert(fbuf != NULL);

    char buffer[1024];
    for(unsigned i = 0; i < num; i++){
        *(float*)(buffer + i*sizeof(float)) = fbuf[i];
    }
    sendBytes(buffer, num*sizeof(float), socket_fd);
}

void sendInts(int *ibuf, unsigned num, int socket_fd){
    assert(ibuf != NULL);

    char buffer[1024];
    for(unsigned i = 0; i < num; i++){
        sprintf(buffer, "%d\n\r", ibuf[i]);
        sendBytes(buffer, strlen(buffer), socket_fd);
    }
}


void receiveFloats(float *fbuf, unsigned num, int socket_fd){
    assert(fbuf != NULL);

    char buffer[1024] = {0};
    int bytes_received = 0;

    while(bytes_received < (int)(num*sizeof(float))){
        int size = recv(socket_fd, buffer+bytes_received, num*sizeof(float)-bytes_received, 0);
        if(size <= 0){ break; }
        bytes_received += size;
    }
    assert(bytes_received == (int)(num*sizeof(float)));
    
    for(unsigned i = 0; i < num; i++){
        fbuf[i] = *(float*)(buffer+sizeof(float)*i);
    }
}

void receiveInts(int *ibuf, unsigned num, int socket_fd){
    assert(ibuf != NULL);

    char buffer[1024] = {0};
    int bytes_received = 0;

    while(bytes_received < (int)(num*sizeof(int))){
        int size = recv(socket_fd, buffer+bytes_received, num*sizeof(int)-bytes_received, 0);
        if(size <= 0){ break; }
        bytes_received += size;
    }
    assert(bytes_received == (int)(num*sizeof(int)));

    for(unsigned i = 0; i < num; i++){
        ibuf[i] = *(int*)(buffer+sizeof(int)*i);
    }
}

void receiveBytes(char *buf, unsigned num, int socket_fd){
    int bytes_received = 0;

    while(bytes_received < (int)num){
        int size = recv(socket_fd, buf+bytes_received, num-bytes_received, 0);
        if(size <= 0){ break; }
        bytes_received += size;
    }
    assert(bytes_received == (int)num);
}

void receiveArmFloats(float *fbuf, unsigned num, int socket_fd){
    assert(fbuf != NULL);

	char buffer[32];
    for(unsigned i = 0; i < num; i++) {
        memset(buffer, 0, 32);
        char tmp = '\0';
        int cur_pos = 0;

        while(1){
		    int bytes_received = recv(socket_fd, &tmp, 1, 0);
		    if(bytes_received <= 0) {
			    std::cerr << "Could not recive arm floats" << std::endl;
			    return;    
            }
            if(tmp == '|'){
                buffer[cur_pos] = '\n';
                break;
            }

            buffer[cur_pos] = tmp;
            cur_pos++;
            
        }
        fbuf[i] = atof(buffer);
	}
}

bool handleCommand(int command_id, int reply_socket_fd){
    float r = 1.0f;
    
    switch(command_id){

      case ARM_PROTOCOL_QUIT:
        std::cout << "-disconnecting" << std::endl;
        return false;

      case ARM_PROTOCOL_MOVE_TO_POINT:
      {
        std::cout << "-move to point:" << std::endl;
        float apmtp_args[6];
        receiveFloats(apmtp_args, 6, reply_socket_fd);
        for(unsigned i = 0; i < 6; i++){
            std::cout << " " << apmtp_args[i];
        }
        std::cout << std::endl;

        sendInts(&command_id, 1, arm_socket_fd);
        sendFloats(apmtp_args, 6, arm_socket_fd);

        sendRawFloats(&r, 1, reply_socket_fd);
      }  
        break;

      case ARM_PROTOCOL_MOVE_JOINT_REL:
      {
        std::cout << "-move joint rel:" << std::endl;
        int joint_num_rel;
        float degrees_rel;
        receiveInts(&joint_num_rel, 1, reply_socket_fd);
        receiveFloats(&degrees_rel, 1, reply_socket_fd);
        std::cout << " " << joint_num_rel << " " << degrees_rel << std::endl;

        sendInts(&command_id, 1, arm_socket_fd);
        sendInts(&joint_num_rel, 1, arm_socket_fd);
        sendFloats(&degrees_rel, 1, arm_socket_fd);

        sendRawFloats(&r, 1, reply_socket_fd);
      }
        break;

      case ARM_PROTOCOL_MOVE_JOINT_ABS:
      {
        std::cout << "-move joint abs:" << std::endl;
        int joint_num_abs;
        float degrees_abs;
        receiveInts(&joint_num_abs, 1, reply_socket_fd);
        receiveFloats(&degrees_abs, 1, reply_socket_fd);
        std::cout << " " << joint_num_abs << " " << degrees_abs << std::endl;

        sendInts(&command_id, 1, arm_socket_fd);
        sendInts(&joint_num_abs, 1, arm_socket_fd);
        sendFloats(&degrees_abs, 1, arm_socket_fd);

        sendRawFloats(&r, 1, reply_socket_fd);
      }
        break;

      case ARM_PROTOCOL_REQUEST_CURRENT_POS:
      {
        std::cout << "-request current pos:" << std::endl;
        sendInts(&command_id, 1, arm_socket_fd);
        float aprcp_args[6];
        receiveArmFloats(aprcp_args, 6, arm_socket_fd);
        for(unsigned i = 0; i < 6; i++){
            std::cout << " " << aprcp_args[i];
        }
        std::cout << std::endl; 
        sendRawFloats(aprcp_args, 6, reply_socket_fd); 
      }
        break;

      case ARM_PROTOCOL_REQUEST_CURRENT_JOINTS:
      {
        std::cout << "-request current joints:" << std::endl;
        sendInts(&command_id, 1, arm_socket_fd);
        float aprcj_args[6];
        receiveArmFloats(aprcj_args, 6, arm_socket_fd);
        for(unsigned i = 0; i < 6; i++){
            std::cout << " " << aprcj_args[i];
        }
        std::cout << std::endl;
        sendRawFloats(aprcj_args, 6, reply_socket_fd); 
      }
        break;

      case ARM_PROTOCOL_REQUEST_REQUIRED_JOINTS:
      {
        std::cout << "-request required joints:" << std::endl;

        float aprrj_args[6];
        receiveFloats(aprrj_args, 6, reply_socket_fd);
        for(unsigned i = 0; i < 6; i++){
            std::cout << aprrj_args[i] << " ";
        }
        std::cout << std::endl;

        sendInts(&command_id, 1, arm_socket_fd);
        sendFloats(aprrj_args, 6, arm_socket_fd);

        float aprrj_result[6];
        receiveArmFloats(aprrj_result, 6, arm_socket_fd);
        for(unsigned i = 0; i < 6; i++){
            std::cout << aprrj_result[i] << " ";
        }
        std::cout << std::endl;

        sendRawFloats(aprrj_result, 6, reply_socket_fd);
      }
        break;

      case ARM_PROTOCOL_MOVE_FREEZE:
      {
        std::cout << "-freeze" << std::endl;
        sendInts(&command_id, 1, arm_socket_fd);
      }
        break;

      case ARM_PROTOCOL_MOVE_MULTIPLE_JOINTS_ABS:
      {
        std::cout << "-move multiple joints abs:" << std::endl;
        float apmmja_args[6];
        receiveFloats(apmmja_args, 6, reply_socket_fd);
        for(unsigned i = 0; i < 6; i++){
            std::cout << " " << apmmja_args[i];
        }
        std::cout << std::endl;

        sendInts(&command_id, 1, arm_socket_fd);
        sendFloats(apmmja_args, 6, arm_socket_fd);

        sendRawFloats(&r, 1, reply_socket_fd);
      }
        break;

      case ARM_PROTOCOL_ROTATION:
      {
        std::cout << "-rotation:" << std::endl;
        char byte_buffer[sizeof(int) + 4*sizeof(float)];
        receiveBytes(byte_buffer, sizeof(int) + 4*sizeof(float), reply_socket_fd);

        int mode; 
        float angle;
        float p1, p2, p3;
        
        mode  = *(int*)byte_buffer;
        angle = *(float*)(byte_buffer + sizeof(int));
        p1 = *(float*)(byte_buffer + sizeof(int) + sizeof(float));
        p2 = *(float*)(byte_buffer + sizeof(int) + 2*sizeof(float));
        p3 = *(float*)(byte_buffer + sizeof(int) + 3*sizeof(float));
        std::cout << "(" << mode << "," << angle << ") "
                  << p1 << " " << p2 << " " << p3 << std::endl;

        float float_args[4];

        float_args[0] = angle; 
        float_args[1] = p1; 
        float_args[2] = p2; 
        float_args[3] = p3;

        sendInts(&command_id, 1, arm_socket_fd);
        sendInts(&mode, 1, arm_socket_fd);       
        sendFloats(float_args, 4, arm_socket_fd);
      }
        break;

      case ARM_GRIPPER_CLOSE:
      {
        std::cout << "-gripper close:" << std::endl;
        float hand_close_amount;
        receiveFloats(&hand_close_amount, 1, reply_socket_fd);
        
        int speed;
        receiveInts(&speed, 1, reply_socket_fd);
        if(speed == -1){ speed = HAND_SPEED; }

        closeHand(hand_close_amount, speed);
        sendRawFloats(&r, 1, reply_socket_fd);
      }
        break;

      case ARM_GRIPPER_SOFT_CLOSE:
      {
        std::cout << "-gripper soft close:" << std::endl;
        float close_start_amount;
        receiveFloats(&close_start_amount, 1, reply_socket_fd);

        softCloseHand(close_start_amount);
        sendRawFloats(&r, 1, reply_socket_fd);
      }
        break;

      case ARM_GRIPPER_OPEN:
      {
        std::cout << "-gripper open:" << std::endl;
        float hand_open_amount;
        receiveFloats(&hand_open_amount, 1, reply_socket_fd);
        
        int speed;
        receiveInts(&speed, 1, reply_socket_fd);
        if(speed == -1){ speed = HAND_SPEED; }

        openHand(hand_open_amount, speed);
        sendRawFloats(&r, 1, reply_socket_fd);
      }
        break;

      case ARM_GRIPPER_RELEASE:
      {
        std::cout << "-gripper release:" << std::endl;
        int speed;
        receiveInts(&speed, 1, reply_socket_fd);
        if(speed == -1){ speed = HAND_SPEED; }

        releaseHand(speed);
        sendRawFloats(&r, 1, reply_socket_fd);
      }
        break;

      default:
        std::cerr << "Unknown arm command id: " << command_id << std::endl;
        break;
    };

    return true;

}

void clientHandler(int client_socket_fd){
    char buffer[4] = {0};

    if(send(client_socket_fd, "ARM_SERVER", 10, 0) != 10){
        return;
    }

    while(true){
        int size = recv(client_socket_fd, buffer, 4, 0);
        if(size <= 0){ break; }
        
        int cmd = *(int*)buffer;
        if(!handleCommand(cmd, client_socket_fd)){ break; }
    }  
}


int main(int argc, char **argv){
    initialiseNeck();
    initialiseHand();

    openHand(1.0f, HAND_SPEED);

    if(!connectToArm()){ return 1; } // We must connect to the arm before we do anything.

    std::cout << "Connection Established" << std::endl;

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


