
#ifndef _Arm_H_
#define _Arm_H_

#include <vector>

class Arm {
  public:
    static Arm *getArm(void);

    Arm(){};
    virtual ~Arm(){};

    virtual std::vector<float> requestCurrentPos(void) = 0;
    virtual std::vector<float> requestCurrentAngles(void) = 0;
    virtual std::vector<float> requestRequiredAngles(float x, float y, float z,
                                                     float rx, float ry, float rz) = 0;

    virtual bool moveToPoint(float x, float y, float z,
                             float rx, float ry, float rz) = 0;
    virtual bool amAtPoint(float x, float y, float z,
                           float rx, float ry, float rz, float delta) = 0;

    virtual bool moveJointRel(int joint_number, float theta) = 0;
    virtual bool moveJointAbs(int joint_number, float theta) = 0;
    virtual bool moveJointsAbs(std::vector<float> theta_arr) = 0;
    virtual bool moveRotation(int mode, float angle, float p1, float p2, float p3) = 0;

    virtual bool freeze(void) = 0;

    virtual bool closeHand(float amount=1.0f) = 0;
    virtual bool openHand(float amount=1.0f) = 0;

};

#endif

