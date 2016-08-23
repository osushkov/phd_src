
#include "IKFitnessFunctionNearest.h"
#include "IKCommon.h"
#include "../Util/Geometry.h"

IKFitnessFunctionNearest::IKFitnessFunctionNearest(std::vector<Vector3D> target_pose) :
    target_pose(target_pose) {

}

float IKFitnessFunctionNearest::eval(std::vector<float> &params) {
    assert(params.size() == NUM_ARM_JOINTS);

    std::vector<float> cur_joint_angles = convertParamsToAngles(params);
    if (!forward_kinematics.checkJointLimits(cur_joint_angles)) {
        return 1000000.0f;
    }

    std::vector<Vector3D> full_pose = forward_kinematics.getArmPoint(cur_joint_angles);

    float diff = 0.0f;
    for(unsigned i = 0; i < full_pose.size(); i++){
    	diff += (full_pose[i]-target_pose[i]).length2();
    }

    return diff;
}
