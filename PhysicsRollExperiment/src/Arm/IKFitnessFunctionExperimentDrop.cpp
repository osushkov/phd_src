/*
 * IKFitnessFunctionExperimentDrop.cpp
 *
 *  Created on: 30/09/2010
 *      Author: osushkov
 */

#include "IKFitnessFunctionExperimentDrop.h"
#include "IKCommon.h"
#include "../Util/Common.h"
#include "../Util/Geometry.h"
#include "../Object/ObjectGrip.h"

#include <stdio.h>

#define _USE_MATH_DEFINES
#include <math.h>

IKFitnessFunctionExperimentDrop::IKFitnessFunctionExperimentDrop(Transform gripper_to_object_pose,
																 Vector3D experiment_up_vector,
																 float experiment_height) :

	gripper_to_object_pose(gripper_to_object_pose), 
    experiment_up_vector(experiment_up_vector), 
    experiment_height(experiment_height) {

}

float IKFitnessFunctionExperimentDrop::eval(std::vector<float> &params){
    assert(params.size() == NUM_ARM_JOINTS);

    std::vector<float> cur_joint_angles = convertParamsToAngles(params);
    if(!forward_kinematics.checkJointLimits(cur_joint_angles)){
        return 1000000.0f;
    }

    std::vector<Vector3D> full_pose = forward_kinematics.getArmPoint(cur_joint_angles);
    Transform object_pose = ObjectGrip::findObjectPose(full_pose, gripper_to_object_pose);

    Matrix3 inv_mat;
    inv_mat.isInverse(object_pose.mat);
    Vector3D object_up = inv_mat*Vector3D(0.0f, 0.0f, 1.0f);

    std::pair<Vector3D,Vector3D> workspace_limits(Vector3D(-22.0f, 33.0f, 0.0f), Vector3D(22.0f, 66.0f, 0.0f));

    float workspace_penalty = 0.0f;
    if(object_pose.shift.x < workspace_limits.first.x || object_pose.shift.x > workspace_limits.second.x ||
       object_pose.shift.y < workspace_limits.first.y || object_pose.shift.y > workspace_limits.second.y){
        workspace_penalty = 100000.0f;
    }

    float result = (object_pose.shift.z - experiment_height)*(object_pose.shift.z - experiment_height) +
    			   fabs(object_up.dotProduct(experiment_up_vector) - 1.0f)*10.0f + workspace_penalty;

    return result;
}
