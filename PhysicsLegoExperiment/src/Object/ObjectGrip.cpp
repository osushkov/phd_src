/*
 * ObjectGrip.cpp
 *
 *  Created on: 28/09/2010
 *      Author: osushkov
 */

#include "ObjectGrip.h"
#include "../Control.h"
#include "../Arm/Arm.h"
#include "../Arm/ArmInverseKinematics.h"
#include "../Arm/IKCommon.h"
#include "../Arm/IKFitnessFunctionExperimentPickup.h"
#include "../Visualisation/SceneRenderer.h"
#include "../Visualisation/ArmRenderObject.h"
#include "../Util/ParallelServer.h"
#include "../Util/Geometry.h"
#include <cassert>
#include <cstdio>
#include <fstream>
#include <queue>

#define _USE_MATH_DEFINES
#include <math.h>

using namespace ObjectGrip;

static std::vector<std::pair<Vector3D, Matrix3> >
getPossibleGripPoses(Vector3D object_pos, std::vector<Vector3D> alignment_axes, std::vector<bool> can_grip_axis){
    assert(alignment_axes.size() == 2 && can_grip_axis.size() == 2);

    object_pos = object_pos + Vector3D(0.0f, 15.0f, 0.0f);
    std::cout << "object pos: ";
    object_pos.print();

    for(unsigned i = 0; i < alignment_axes.size(); i++){
        alignment_axes[i].print();
    }

    std::vector<std::pair<Vector3D, Matrix3> > result;

    for(unsigned i = 0; i < alignment_axes.size(); i++){
        if(!can_grip_axis[i]){
            continue;
        }

        Vector3D approach_axis = alignment_axes[(i+1)%alignment_axes.size()];
        Vector3D grip_axis = alignment_axes[i];

        Vector3D start_point, end_point;
        if((object_pos+approach_axis).length() < (object_pos-approach_axis).length()){
            start_point = object_pos + 0.1f*approach_axis;
            end_point = object_pos - 0.1f*approach_axis;
        }
        else{
            start_point = object_pos - 0.1f*approach_axis;
            end_point = object_pos + 0.1f*approach_axis;
        }

        Vector3D mid_point = 0.5f * (start_point + end_point);
        Vector3D vec = start_point - mid_point;

        static const unsigned num_positions = 5;
        static const unsigned num_orientations = 1;
        for(unsigned j = 0; j <= num_positions; j++){

            Vector3D grip_point; // = start_point + (float)j/(float)num_positions * (end_point - start_point);
            if(j%2 == 0){
                grip_point = mid_point + (float)j/(float)num_positions * vec;
            }
            else{
                grip_point = mid_point - (float)j/(float)num_positions * vec;
            }

            if(grip_point.y >= 460.0f){
                grip_point.y = 460.0f;
            }

            grip_point.y += 10.0f;

            for(unsigned k = 0; k <= num_orientations; k++){
                Vector3D z_axis = end_point - start_point;
                z_axis.normalise();

                Vector3D y_axis = Vector3D(0.0f, 1.0f, 0.0f);

                Vector3D x_axis = y_axis.crossProduct(z_axis);
                x_axis.normalise();

                std::vector<Vector3D> basis_set;
                basis_set.push_back(x_axis);
                basis_set.push_back(y_axis);
                basis_set.push_back(z_axis);

                Matrix3 ori_mat = Geometry::matrixFromBasisVectors(basis_set);
                Matrix3 gripper_rot_mat = Geometry::axisRotationMatrix(z_axis, 7.0f * M_PI/180.0f);
                ori_mat = gripper_rot_mat*ori_mat;

                Matrix3 rot_mat = Geometry::axisRotationMatrix(x_axis, -M_PI/2.0f);//(float)k/(float)num_orientations * 2.0f * M_PI);
                ori_mat = rot_mat*ori_mat;

                result.push_back(std::pair<Vector3D,Matrix3>(grip_point, ori_mat));
            }
        }
    }

    return result;
}


static GripData calculateBestGripPoint(Vector3D world_pos, std::vector<Vector3D> alignment_axes, std::vector<bool> can_grip_axis){

    Vector3D object_centre = Common::worldPointToArmSpace(world_pos);
    for(unsigned i = 0; i < alignment_axes.size(); i++){
	    alignment_axes[i] = 
            Common::worldPointToArmSpace(alignment_axes[i]) - Common::worldPointToArmSpace(Vector3D(0.0f, 0.0f, 0.0f));
    }

    std::vector<std::pair<Vector3D, Matrix3> > possible_grip_poses = 
        getPossibleGripPoses(object_centre, alignment_axes, can_grip_axis);

    GripData result;
    result.success = false;

    ArmInverseKinematics ik;
    for(unsigned i = 0; i < possible_grip_poses.size(); i++){
        IKResult ik_result = ik.performKDLSearch(possible_grip_poses[i].second, possible_grip_poses[i].first);
        if(ik_result.success){
            result.success = true;
            result.arm_joints = ik_result.joints;
            while(result.arm_joints[5] > 91.0f){
                result.arm_joints[5] -= 180.0f;
            }
            while(result.arm_joints[5] < -91.0f){
                result.arm_joints[5] += 180.0f;
            }
            result.root_joints = result.arm_joints;

            return result;
        }
    }

    return result;
}

GripData ObjectGrip::gripObject(Object *object, Transform obj_pose){
	assert(object != NULL);

	std::vector<Vector3D> object_axes = object->getPrimaryAxes();
	for(unsigned i = 0; i < object_axes.size(); i++){
		object_axes[i] =  obj_pose.mat*object_axes[i];
	}

    std::vector<float> vertical_dps;
    for(unsigned i = 0; i < object_axes.size(); i++){
        Vector3D n_axis = object_axes[i];
        n_axis.normalise();
        vertical_dps.push_back(fabs(n_axis.dotProduct(Vector3D(0.0f, 0.0f, 1.0f))));
    }

    assert(object_axes.size() == 3);

    std::vector<bool> can_grip_axis;
    std::vector<Vector3D> alignment_axes;

    static const float MAX_GRIP_SIZE = 10.0f;
    if(vertical_dps[0] < vertical_dps[1] || vertical_dps[0] < vertical_dps[2]){
        can_grip_axis.push_back(2.0f*object_axes[0].length() <= MAX_GRIP_SIZE);
        alignment_axes.push_back(object_axes[0]);
    }
    if(vertical_dps[1] < vertical_dps[0] || vertical_dps[1] < vertical_dps[2]){
        can_grip_axis.push_back(2.0f*object_axes[1].length() <= MAX_GRIP_SIZE);
        alignment_axes.push_back(object_axes[1]);
    }
    if(vertical_dps[2] < vertical_dps[0] || vertical_dps[2] < vertical_dps[1]){
        can_grip_axis.push_back(2.0f*object_axes[2].length() <= MAX_GRIP_SIZE);
        alignment_axes.push_back(object_axes[2]);
    }

    assert(alignment_axes.size() == 2);
    assert(can_grip_axis.size() == 2);

	return calculateBestGripPoint(obj_pose.shift, alignment_axes, can_grip_axis);
}


static Matrix3 getArmOrientationMatrix(std::vector<Vector3D> pose){
	Matrix3 result;
	for(unsigned i = 0; i < 3; i++){
		result(0,i) = pose[i+1].x;
		result(1,i) = pose[i+1].y;
		result(2,i) = pose[i+1].z;
	}
	return result;
}

Transform ObjectGrip::findObjectPose(std::vector<Vector3D> gripper_pose, Transform arm_to_obj_pose){
	Matrix3 cur_arm_ori = getArmOrientationMatrix(gripper_pose);
	Transform ac_result;

	ac_result.shift = cur_arm_ori*arm_to_obj_pose.shift + gripper_pose[0];
	ac_result.mat = cur_arm_ori*arm_to_obj_pose.mat;

	Transform wc_result;
	wc_result.shift = Common::armPointToWorldSpace(ac_result.shift);

	std::vector<Vector3D> oriv;
	oriv.push_back(Common::armPointToWorldSpace(ac_result.shift + Vector3D(ac_result.mat(0,0),ac_result.mat(1,0),ac_result.mat(2,0))) - wc_result.shift);
	oriv.push_back(Common::armPointToWorldSpace(ac_result.shift + Vector3D(ac_result.mat(0,1),ac_result.mat(1,1),ac_result.mat(2,1))) - wc_result.shift);
	oriv.push_back(Common::armPointToWorldSpace(ac_result.shift + Vector3D(ac_result.mat(0,2),ac_result.mat(1,2),ac_result.mat(2,2))) - wc_result.shift);

	for(unsigned i = 0; i < 3; i++){
		oriv[i].normalise();
		wc_result.mat(0, i) = oriv[i].x;
		wc_result.mat(1, i) = oriv[i].y;
		wc_result.mat(2, i) = oriv[i].z;
	}

	wc_result.quaternion = Quaternion(wc_result.mat);
	wc_result.secondary_shift = Vector3D(0.0f, 0.0f, 0.0f);

	return wc_result;
}

Transform ObjectGrip::armToObjectPoseTransform(Transform obj_pose, std::vector<float> grip_arm_joints){
	ArmForwardKinematics forward_kinematics;
	std::vector<Vector3D> grip_pose = forward_kinematics.getArmPoint(grip_arm_joints);

	Matrix3 arm_ori = getArmOrientationMatrix(grip_pose);
	Matrix3 arm_ori_inv;
	arm_ori_inv.isTranspose(arm_ori);

	Transform ac_obj_pose;
	ac_obj_pose.shift = Common::worldPointToArmSpace(obj_pose.shift);

	std::vector<Vector3D> oriv;
	oriv.push_back(Common::worldPointToArmSpace(obj_pose.shift + Vector3D(obj_pose.mat(0,0),obj_pose.mat(1,0),obj_pose.mat(2,0))) - ac_obj_pose.shift);
	oriv.push_back(Common::worldPointToArmSpace(obj_pose.shift + Vector3D(obj_pose.mat(0,1),obj_pose.mat(1,1),obj_pose.mat(2,1))) - ac_obj_pose.shift);
	oriv.push_back(Common::worldPointToArmSpace(obj_pose.shift + Vector3D(obj_pose.mat(0,2),obj_pose.mat(1,2),obj_pose.mat(2,2))) - ac_obj_pose.shift);

	for(unsigned i = 0; i < 3; i++){
		oriv[i].normalise();
		ac_obj_pose.mat(0, i) = oriv[i].x;
		ac_obj_pose.mat(1, i) = oriv[i].y;
		ac_obj_pose.mat(2, i) = oriv[i].z;
	}

	Transform arm_to_obj_pose;
	arm_to_obj_pose.shift = ac_obj_pose.shift - grip_pose[0];
	arm_to_obj_pose.shift = arm_ori_inv * arm_to_obj_pose.shift;

	arm_to_obj_pose.mat = arm_ori_inv*ac_obj_pose.mat;
	arm_to_obj_pose.quaternion = Quaternion(arm_to_obj_pose.mat);
	arm_to_obj_pose.secondary_shift = Vector3D(0.0f, 0.0f, 0.0f);

	return arm_to_obj_pose;
}

static std::vector<float> randomArmVector(void){
    std::vector<float> result;
    for(unsigned i = 0; i < 6; i++){
        result.push_back((float)rand()/(float)RAND_MAX);
    }
    return result;
}
