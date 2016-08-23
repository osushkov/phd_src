
#include "IKFitnessFunctionObjectPickup.h"
#include "IKCommon.h"
#include "../Util/Geometry.h"

#define _USE_MATH_DEFINES
#include <math.h>

IKFitnessFunctionObjectPickup::IKFitnessFunctionObjectPickup(Vector3D object_position,
                                                             std::vector<float> cur_joints) :
    object_position(object_position), cur_joints(cur_joints) {

}

float IKFitnessFunctionObjectPickup::eval(std::vector<float> &params){
    assert(params.size() == NUM_ARM_JOINTS);

    std::vector<float> cur_joint_angles = convertParamsToAngles(params);
    if(!forward_kinematics.checkJointLimits(cur_joint_angles)){
        return 1000000.0f;
    }

    std::vector<Vector3D> full_pose = forward_kinematics.getArmPoint(cur_joint_angles);
    Matrix3 rot_mat = Geometry::axisRotationMatrix(full_pose[3], -25.0f*(float)M_PI/180.0f);
    Vector3D offset_vec = full_pose[1];
    offset_vec = rot_mat * offset_vec;

    Vector3D left_paddle_pos = full_pose[0] + offset_vec;
    Vector3D right_paddle_pos = full_pose[0] - offset_vec;

    float height_diff = (left_paddle_pos.y-full_pose[0].y)*(left_paddle_pos.y-full_pose[0].y) +
                        (right_paddle_pos.y-full_pose[0].y)*(right_paddle_pos.y-full_pose[0].y);
    height_diff *= 10.0f;

    float joints_diff = 0.0f;
    for(unsigned i = 0; i < cur_joint_angles.size(); i++){
        float cur_joints_diff = sqrtf((cur_joint_angles[i]-cur_joints[i])*
                                      (cur_joint_angles[i]-cur_joints[i]));
        if(cur_joints_diff > joints_diff){
            joints_diff = cur_joints_diff;
        }
    }
    joints_diff /= 1000.0f;

    float distance_penalty = 0.0f;
    distance_penalty += (full_pose[0] - object_position).length2();
    distance_penalty += fabs(full_pose[3].y);
    distance_penalty += 5.0f*height_diff;
    distance_penalty += joints_diff;
    distance_penalty += calculateJointStress(params);

    return distance_penalty;// + joint_stress_penalty;
}
