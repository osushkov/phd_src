
#ifndef _DummyArm_H_
#define _DummyArm_H_

#include "Arm.h"
#include <vector>


class DummyArm : public Arm {
  public:
    DummyArm(){};
    ~DummyArm(){};


    std::vector<float> requestCurrentPos(void){
        return std::vector<float>(6);
    }
    std::vector<float> requestCurrentAngles(void){
        return std::vector<float>(6);
    }
    std::vector<float> requestRequiredAngles(float x, float y, float z,
                                             float rx, float ry, float rz){
        return std::vector<float>(6);
    }


    bool moveToPoint(float x, float y, float z,
                     float rx, float ry, float rz){
        return true;
    }

    bool amAtPoint(float x, float y, float z,
                   float rx, float ry, float rz,
                   float delta){
        return true;
    }

    bool moveJointRel(int joint_number, float theta){ return true; }
    bool moveJointAbs(int joint_number, float theta){ return true; }
    bool moveJointsAbs(std::vector<float> theta_arr){ return true; }
    bool moveRotation(int mode, float angle, float p1, float p2, float p3){ return true; }

    bool freeze(void){ return true; }

    bool closeHand(float amount=1.0f){ return true; }
    bool softCloseHand(float initial_amount=0.0f){ return true; }
    bool openHand(float amount=1.0f){ return true; }

};

#endif

