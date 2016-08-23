/*
 * IKFitnessFunctionExperimentPickup.cpp
 *
 *  Created on: 29/09/2010
 *      Author: osushkov
 */

#include "IKFitnessFunctionExperimentPickup.h"
#include "IKCommon.h"
#include "../Util/Geometry.h"
#include "../Util/Quaternion.h"

#define _USE_MATH_DEFINES
#include <math.h>

static unsigned iter = 0;

IKFitnessFunctionExperimentPickup::IKFitnessFunctionExperimentPickup(Vector3D obj_centre,
                                                                     std::vector<Vector3D> alignment_axes,
                                                                     std::vector<bool> can_grip_axis) :
	obj_centre(obj_centre), alignment_axes(alignment_axes), can_grip_axis(can_grip_axis) {
/*
    std::cout << "!!!" << std::endl;
    obj_centre.print();
    for(unsigned i = 0; i < alignment_axes.size(); i++){
        std::cout << can_grip_axis[i] << ": ";
        alignment_axes[i].print();
    }

    getchar();
*/
}

float IKFitnessFunctionExperimentPickup::eval(std::vector<float> &params){
    assert(params.size() == NUM_ARM_JOINTS);
    iter++;

    std::vector<float> cur_joint_angles = convertParamsToAngles(params);
    if(!forward_kinematics.checkJointLimits(cur_joint_angles)){
        std::cout << "join limit" << std::endl;
        return 1000000.0f;
    }

    std::vector<Vector3D> full_pose = forward_kinematics.getArmPoint(cur_joint_angles);
    std::pair<Vector3D,Vector3D> gripper_points = forward_kinematics.getGripperPosition(cur_joint_angles);
    float gripper_height_error = fabs(gripper_points.first.y - gripper_points.second.y);
    
    Vector3D gripper_axis = gripper_points.first - gripper_points.second;
    gripper_axis.normalise();

    float least_alignment_error = 10000.0f;
    for(unsigned i = 0; i < alignment_axes.size(); i++){
        if(!can_grip_axis[i]){
            continue;
        }

        Vector3D obj_main_axis = alignment_axes[(i+1)%alignment_axes.size()];
        Vector3D obj_grip_axis = alignment_axes[i];
        obj_grip_axis.normalise();

        float gripper_ori_error = (1.0f - fabs(gripper_axis.dotProduct(obj_grip_axis)))*5.0f;

        Vector3D start = obj_centre + 0.5f*obj_main_axis;
        Vector3D end = obj_centre - 0.5f*obj_main_axis;
        float distT = Geometry::linePointDistanceT(start, end, full_pose[0]);

        float dist_error = 0.0f;
        if(distT < 0.0f){
    	    dist_error = (full_pose[0] - start).length();
        }
        else if(distT > 1.0f){
    	    dist_error = (full_pose[0] - end).length();
        }
        else{
    	    Vector3D p = start + distT*(end - start);
    	    dist_error = (full_pose[0] - p).length();
        }

        float alignment_error = dist_error + gripper_ori_error;
        if(alignment_error < least_alignment_error){
            least_alignment_error = alignment_error;
        }
    }

    if(debug){
        
    }
    return least_alignment_error; //dist_error + gripper_height_error + gripper_ori_error;// + axis_error;
}

