

#include "Arm.h"
#include "RemoteArm.h"
#include "DummyArm.h"
#include "../Settings.h"

static Arm *my_arm = NULL;

Arm* Arm::getArm(void){
    if(my_arm != NULL){ return my_arm; }

    if(Settings::instance().getIntValue("general", "live_arm")){
        my_arm = 
            new RemoteArm(Settings::instance().getStringValue("general", "arm_address"),
                          Settings::instance().getIntValue("general", "arm_port"));
    } 
    else {
        my_arm = new DummyArm();
    }

    return my_arm;
}

