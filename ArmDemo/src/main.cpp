
#include <iostream>
#include <vector>
#include <cmath>
#include "RemoteArm.h"

void bringToStartPos(RemoteArm &remote_arm, float rotation){
    float x = 200.0f;
    float y = 310.0f;
    float z = 130.0f;

    remote_arm.moveToPoint(x, y, z, -170.0f, 47.0f, -144.0f);
    remote_arm.closeHand(rotation);
}

void bringToEndPos(RemoteArm &remote_arm, float rotation){
    float x = 415.0f;
    float y = 310.0f;
    float z = 240.0f;

    remote_arm.moveToPoint(x, y, z, -170.0f, 47.0f, -144.0f);
    remote_arm.closeHand(rotation);
}

int main(int argc, char **argv){

    RemoteArm remote_arm("humanoid-leftbrain", 5006);
    sleep(1);

    //remote_arm.moveJointRel(5, -10);
    std::cout << "Moving joints" << std::endl;

    remote_arm.closeHand();

    float theta = 0.0f;
    remote_arm.moveToPoint(300, 350, 150, -10.0, 63.0f, 0.0f);
 /*
    for(unsigned i = 0; i < 4; i++){
        bringToStartPos(remote_arm, theta); theta += 0.166f;
        std::cout << i << ": Waiting to bring into Start Pos..." << std::endl;
        getchar();

        bringToEndPos(remote_arm, theta); theta += 0.166f;
        std::cout << i << ": Waiting to bring into End Pos..." << std::endl;
        getchar();
    }*/

/*
    std::vector<float> cur_jnts = remote_arm.requestCurrentPos();
    
    for(unsigned i = 0; i < cur_jnts.size(); i++){
        std::cout << cur_jnts[i] << std::endl;
    }
*/
    return 0;


    std::cout << "Beginning waving" << std::endl;
    for(unsigned num = 0; num  < 2; num++){
    for(int i = 0; i <= 3; i++){
        float theta = i * 360.0f/10.0f;
        float y = 320.0f;

        float x = 290.0f + sin(theta * M_PI/180.0f)*100.0f;
        float z = 200.0f + cos(theta * M_PI/180.0f)*100.0f;

        std::vector<float> jnts = remote_arm.requestRequiredAngles(x, y, z, -166.0f, -43.0f, -23.0f);
    
        jnts.back() = -180.0f + theta/2.0f;

        sleep(1);
        remote_arm.moveJointsAbs(jnts);
        sleep(3);

        if(i == 0 && num == 0){
            sleep(15);
        } 
    }
    }

    return 0;
}

