
#include "RemoteArm.h"
#include "ArmProtocol.h"

#include <iostream>
#include <vector>
#include <sstream>
#include <signal.h>
#include <cassert>
#include <cstring>

#ifdef _WIN32
#include <winsock.h>
#else
#include <unistd.h>
#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#endif

#define _USE_MATH_DEFINES
#include <math.h>

#include "../Settings.h"

#define DENSO_BUF_SIZE 32


RemoteArm::RemoteArm(std::string host_address, int port) :
    host_address(host_address), port(port), connected(false) {

    connectToArm();
}

RemoteArm::~RemoteArm(){
    disconnectFromArm();
}

std::vector<float> RemoteArm::requestCurrentPos(void){
    char buffer[1024] = {0};
    *(int*)buffer = ARM_PROTOCOL_REQUEST_CURRENT_POS;
    armSendRaw(buffer, sizeof(int));

    return receiveFloats(6);
}

std::vector<float> RemoteArm::requestCurrentAngles(void){
    char buffer[1024] = {0};

    *(int*)buffer = ARM_PROTOCOL_REQUEST_CURRENT_JOINTS;
    armSendRaw(buffer, sizeof(int));

    return receiveFloats(6);
}

std::vector<float> RemoteArm::requestRequiredAngles(float x, float y, float z, float rx, float ry, float rz){
    char buffer[1024];

    *(int*)buffer = ARM_PROTOCOL_REQUEST_REQUIRED_JOINTS;
    armSendRaw(buffer, sizeof(int));

    *(float*)buffer = x;
    *(float*)(buffer + sizeof(float)) = y;
    *(float*)(buffer + 2*sizeof(float)) = z;
    *(float*)(buffer + 3*sizeof(float)) = rx;
    *(float*)(buffer + 4*sizeof(float)) = ry;
    *(float*)(buffer + 5*sizeof(float)) = rz;
    armSendRaw(buffer, 6*sizeof(float));

    return receiveFloats(6);
}

bool RemoteArm::moveToPoint(float x, float y, float z, float rx, float ry, float rz){
    char buffer[1024];

    *(int*)buffer = ARM_PROTOCOL_MOVE_TO_POINT;
    armSendRaw(buffer, sizeof(int));

    *(float*)buffer = x;
    *(float*)(buffer + sizeof(float)) = y;
    *(float*)(buffer + 2*sizeof(float)) = z;
    *(float*)(buffer + 3*sizeof(float)) = rx;
    *(float*)(buffer + 4*sizeof(float)) = ry;
    *(float*)(buffer + 5*sizeof(float)) = rz;
    armSendRaw(buffer, 6*sizeof(float));
    receiveFloats(1);

    std::vector<float> arm_pos;
    arm_pos.push_back(x);
    arm_pos.push_back(y);
    arm_pos.push_back(z);
    arm_pos.push_back(rx);
    arm_pos.push_back(ry);
    arm_pos.push_back(rz);
    if(armVectorDist(arm_pos, requestCurrentPos()) > 10.0f){
#ifdef _WIN32
        Sleep(10);
#else
    	usleep(10000);
#endif
        return moveToPoint(x, y, z, rx, ry, rz);
    }

    return true;
}

bool RemoteArm::amAtPoint(float x, float y, float z, float rx, float ry, float rz, float delta){
    std::vector<float> wanted_vec;
    wanted_vec.push_back(x);
    wanted_vec.push_back(y);
    wanted_vec.push_back(z);
    wanted_vec.push_back(rx);
    wanted_vec.push_back(ry);
    wanted_vec.push_back(rz);
    return armVectorDist(wanted_vec, requestCurrentPos()) < delta;
}

bool RemoteArm::moveJointRel(int joint_number, float dtheta){
    char buffer[1024];

    *(int*)buffer = ARM_PROTOCOL_MOVE_JOINT_REL;
    armSendRaw(buffer, sizeof(int));

    *(int*)buffer = joint_number;
    *(float*)(buffer+sizeof(int)) = dtheta;
    armSendRaw(buffer, sizeof(int)+sizeof(float));

    receiveFloats(1);
    return true;
}

bool RemoteArm::moveJointAbs(int joint_number, float theta){
    char buffer[1024];

    *(int*)buffer = ARM_PROTOCOL_MOVE_JOINT_ABS;
    armSendRaw(buffer, sizeof(int));

    *(int*)buffer = joint_number;
    *(float*)(buffer+sizeof(int)) = theta;
    armSendRaw(buffer, sizeof(int)+sizeof(float));

    receiveFloats(1);
    return true;
}

bool RemoteArm::moveJointsAbs(std::vector<float> theta_arr){
    if(theta_arr.size() != 6){
        std::cerr << "Move Joints Abs failed: incorrect number of args (6 needed)"
                  << std::endl;
        return false;
    }

    char buffer[1024];

    *(int*)buffer = ARM_PROTOCOL_MOVE_MULTIPLE_JOINTS_ABS;
    armSendRaw(buffer, sizeof(int));

    for(unsigned i = 0; i < 6; i++){
        *(float*)(buffer + sizeof(float)*i) = theta_arr[i];
    }
    armSendRaw(buffer, 6*sizeof(float));
    receiveFloats(1);

    if(armVectorDist(theta_arr, requestCurrentAngles()) > 10.0f){
#ifdef _WIN32
        Sleep(10);
#else
        usleep(10000);
#endif
        return moveJointsAbs(theta_arr);
    }

    return true;
}

bool RemoteArm::moveRotation(int mode, float angle, float p1, float p2, float p3){
    char buffer[1024];

    *(int*)buffer = ARM_PROTOCOL_ROTATION;
    armSendRaw(buffer, sizeof(int));

    *(int*)buffer = mode;
    *(float*)(buffer + sizeof(int)) = angle;
    *(float*)(buffer + sizeof(int) + sizeof(float))   = p1;
    *(float*)(buffer + sizeof(int) + 2*sizeof(float)) = p2;
    *(float*)(buffer + sizeof(int) + 3*sizeof(float)) = p3;
    armSendRaw(buffer, sizeof(int) + 4*sizeof(float));

    return true;
}

bool RemoteArm::freeze(void){
    char buffer[1024];
    *(int*)buffer = ARM_PROTOCOL_MOVE_FREEZE;
    armSendRaw(buffer, sizeof(int));
    return true;
}

bool RemoteArm::closeHandDistance(float cm, int speed){
    static const unsigned grip_map_size = 12;
    static const std::pair<float,float> grip_map[] = {
        std::pair<float,float>(0.0f, 12.3f),
        std::pair<float,float>(0.1f, 12.0f),
        std::pair<float,float>(0.2f, 11.0f),
        std::pair<float,float>(0.3f, 10.1f),
        std::pair<float,float>(0.4f, 9.2f),
        std::pair<float,float>(0.5f, 8.0f),
        std::pair<float,float>(0.6f, 6.8f),
        std::pair<float,float>(0.7f, 5.5f),
        std::pair<float,float>(0.8f, 4.0f),
        std::pair<float,float>(0.9f, 2.8f),
        std::pair<float,float>(0.95f, 2.0f),
        std::pair<float,float>(1.0f, 1.1f),
    };

    if(cm > grip_map[0].second){
        return closeHand(grip_map[0].first, speed);
    }

    for(unsigned i = 0; i < grip_map_size-1; i++){
        if(cm <= grip_map[i].second && cm >= grip_map[i+1].second){
            float x = (cm - grip_map[i].second)/(grip_map[i+1].second - grip_map[i].second);
            float amount = grip_map[i].first + x*(grip_map[i+1].first - grip_map[i].first);
            return closeHand(amount, speed);
        }
    }

    return closeHand(1.0f, speed);
}

bool RemoteArm::softCloseHand(float initial_amount){
    char buffer[1024];
    *(int*)buffer = ARM_GRIPPER_SOFT_CLOSE;
    armSendRaw(buffer, sizeof(int));

    *(float*)buffer = initial_amount;
    armSendRaw(buffer, sizeof(float));

    receiveFloats(1);
    return true;
}

bool RemoteArm::closeHand(float amount, int speed){
    char buffer[1024];
    *(int*)buffer = ARM_GRIPPER_CLOSE;
    armSendRaw(buffer, sizeof(int));

    *(float*)buffer = amount;
    armSendRaw(buffer, sizeof(float));

    *(int*)buffer = speed;
    armSendRaw(buffer, sizeof(int));

    receiveFloats(1);
    return true;
}

bool RemoteArm::openHand(float amount, int speed){
    char buffer[1024];
    *(int*)buffer = ARM_GRIPPER_OPEN;
    armSendRaw(buffer, sizeof(int));

    *(float*)buffer = amount;
    armSendRaw(buffer, sizeof(float));

    *(int*)buffer = speed;
    armSendRaw(buffer, sizeof(int));

    receiveFloats(1);
    return true;
}

bool RemoteArm::releaseHand(int speed){
    char buffer[1024];
    *(int*)buffer = ARM_GRIPPER_RELEASE;
    armSendRaw(buffer, sizeof(int));

    *(int*)buffer = speed;
    armSendRaw(buffer, sizeof(int));

    receiveFloats(1);
    return true;
}

bool RemoteArm::connectToArm(void){
    struct sockaddr_in serv_addr;
    struct hostent *he;

    if((socket_fd = socket(PF_INET, SOCK_STREAM, 0)) == -1){
        return false;
    }

    if((he = gethostbyname(host_address.c_str())) == NULL) {
        return false;
	}

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(port);
	serv_addr.sin_addr = *((struct in_addr*) he->h_addr);
	memset(serv_addr.sin_zero, '\0', sizeof(serv_addr.sin_zero));

    std::cout << "Connecting to Arm Server: " << host_address << " " << port << std::endl;
	if((connect(socket_fd, (struct sockaddr*) &serv_addr, sizeof(serv_addr))) == -1) {
		std::cerr << "Could not connect to arm at: " << host_address << ":"
                  << port << std::endl;
		return false;
	}
    std::cout << "Connected to Arm Server" << std::endl;

	char buffer[DENSO_BUF_SIZE] = {0};
	int bytes_received = recv(socket_fd, buffer, DENSO_BUF_SIZE, 0);

	if(bytes_received < 1) {
	    std::cerr << "Could not receive init buffer from: "	 << host_address << ":"
                  << port << std::endl;
        return false;
	}

    std::cout << "Connected" << std::endl;
    connected = true;
    return true;
}

bool RemoteArm::disconnectFromArm(void){
    int command = ARM_PROTOCOL_QUIT;
    armSendRaw((char*)&command, sizeof(int));
#ifdef _WIN32
    closesocket(socket_fd);
#else
    close(socket_fd);
#endif
    return true;
}

bool RemoteArm::armSendRaw(char* buffer, int length){
	if(send(socket_fd, buffer, length, 0) != length){
        std::cerr << "Could not send raw command" << std::endl;
        return false;
    }
    else{
        return true;
    }
}

std::vector<float> RemoteArm::receiveFloats(unsigned num){
    std::vector<float> result;
    char buffer[1024] = {0};

    int bytes_received = 0;
    while(bytes_received < (int)(num*sizeof(float))){
        int size = recv(socket_fd, buffer+bytes_received, num*sizeof(float)-bytes_received, 0);
        if(size <= 0){ break; }
        bytes_received += size;
    }
    assert(bytes_received == (int)(num*sizeof(float)));

    for(unsigned i = 0; i < num; i++){
        result.push_back(*(float*)(buffer+sizeof(float)*i));
    }

    return result;
}

float RemoteArm::armVectorDist(std::vector<float> v1, std::vector<float> v2){
    assert(v1.size() == v2.size());

    float result = 0.0f;
    for(unsigned i = 0; i < v1.size(); i++){
        result += (v1[i]-v2[i])*(v1[i]-v2[i]);
    }
    result = sqrtf(result);
    return result;
}

