
#include "IKFitnessFunctionNearest.h"
#include "IKCommon.h"
#include "../Util/Geometry.h"

IKFitnessFunctionNearest::IKFitnessFunctionNearest(std::vector<Vector3D> target_pose,
                                                   std::vector<float> cur_joints) :
    target_pose(target_pose), cur_joints(cur_joints) {

}

float IKFitnessFunctionNearest::eval(std::vector<float> &params, float iter_fraction) {
    assert(params.size() == NUM_ARM_JOINTS);

    std::vector<float> cur_joint_angles = convertParamsToAngles(params);
    if (!forward_kinematics.checkJointLimits(cur_joint_angles)) {
        return 1000000.0f;
    }

    std::vector<Vector3D> full_pose = forward_kinematics.getArmPoint(cur_joint_angles);

    float joints_diff = 0.0f;
    for (unsigned i = 0; i < cur_joint_angles.size(); i++) {
        joints_diff +=
            sqrtf((cur_joint_angles[i]-cur_joints[i])*(cur_joint_angles[i]-cur_joints[i]));
    }
    joints_diff /= 1000.0f;

    float distance_penalty = 0.0f;
    for(unsigned i = 0; i < full_pose.size(); i++){
        distance_penalty += (full_pose[i] - target_pose[i]).length();
    }
    distance_penalty += joints_diff;

    return distance_penalty;
}
