
#ifndef _RemoteArm_H_
#define _RemoteArm_H_

#include <string>
#include <vector>

class RemoteArm {
  public:
    RemoteArm(std::string host_address, int port);
    ~RemoteArm();

    std::vector<float> requestCurrentPos(void);
    std::vector<float> requestCurrentAngles(void);
    std::vector<float> requestRequiredAngles(float x, float y, float z, float rx, float ry, float rz);

    bool moveToPoint(float x, float y, float z, float rx, float ry, float rz);
    bool moveJointRel(int joint_number, float theta);
    bool moveJointAbs(int joint_number, float theta);
    bool moveJointsAbs(std::vector<float> theta_arr);
    bool moveRotation(int mode, float angle, float p1, float p2, float p3);

    bool freeze(void);

    bool closeHand(float amount=1.0f);
    bool openHand(float amount=1.0f);

  private:
    std::string host_address;
    int port;

    int socket_fd;


    bool connectToArm(void);
    bool disconnectFromArm(void);
    bool armSendRaw(char *buffer, int length);
    std::vector<float> receiveFloats(unsigned num);

};


#endif

