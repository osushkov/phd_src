/*
 * ObjectExperimentTransfer.cpp
 *
 *  Created on: 30/09/2010
 *      Author: osushkov
 */

#include "ObjectExperimentTransfer.h"
#include "ObjectGrip.h"
#include "../SuperQuadric.h"
#include "../Util/Geometry.h"
#include "../Arm/ArmInverseKinematics.h"
#include "../Arm/IKFitnessFunctionExperimentDrop.h"
#include "../Visualisation/SceneRenderer.h"
#include "../Visualisation/ArmRenderObject.h"
#include "../Visualisation/SuperQuadricRenderObject.h"
#include "../Control.h"
#include <cassert>
#include <cstdio>

#define _USE_MATH_DEFINES
#include <math.h>


static Vector3D generateRandomWorspacePosition(float height){
    static const std::pair<Vector3D,Vector3D> workspace_limits(Vector3D(-22.0f, 33.0f, 0.0f), Vector3D(22.0f, 66.0f, 0.0f));

    Vector3D result;
    result.x = (float)rand()/(float)RAND_MAX * (workspace_limits.second.x - workspace_limits.first.x) + workspace_limits.first.x;
    result.y = (float)rand()/(float)RAND_MAX * (workspace_limits.second.y - workspace_limits.first.y) + workspace_limits.first.y;
    result.z = height;

    return result;
}


static std::vector<std::pair<Vector3D, Matrix3> >
getPossibleDropPoses(Transform arm_to_obj_pose, Vector3D experiment_up_vector, float final_height){
    std::vector<std::pair<Vector3D, Matrix3> > result;

    static const unsigned num_positions = 20;
    static const unsigned num_orientations = 30;

    for(unsigned i = 0; i < num_positions; i++){
        Vector3D position = Common::worldPointToArmSpace(generateRandomWorspacePosition(final_height));

        Matrix3 orientation = Geometry::getMatrixFromTo(experiment_up_vector, Vector3D(0.0f, -1.0f, 0.0f));
        Matrix3 inv_mat;
        inv_mat.isInverse(arm_to_obj_pose.mat);

        orientation = orientation*inv_mat;
        position = position - orientation*arm_to_obj_pose.shift;

        for(unsigned j = 0; j < num_orientations; j++){
            float angle = (float)j/(float)num_orientations * 2.0f * M_PI;
            Matrix3 rot_mat = Geometry::axisRotationMatrix(Vector3D(0.0f, 1.0f, 0.0f), angle);

            result.push_back(std::pair<Vector3D,Matrix3>(position, rot_mat*orientation));
        }
    }

    return result;
}

std::vector<float> ObjectExperimentTransfer::transferObject(Transform arm_to_obj_pose,
                                                            float box_bottom_offset,
                                                            float experiment_height, 
                                                            float experiment_angle,
                                                            float ramp_xoffset,
                                                            Vector3D ramp_normal,
                                                            std::vector<float> cur_joints,
															float &error,
                                                            float &start_y){


    float theta = acosf(ramp_normal.dotProduct(Vector3D(0.0f, 0.0f, 1.0f)));
    Vector3D start_pos = Vector3D(-experiment_height/tanf(theta), 30.0f, experiment_height + GROUND_HEIGHT);
    start_pos = start_pos + box_bottom_offset*ramp_normal;

    Matrix3 rot_mat = Geometry::axisRotationMatrix(ramp_normal, experiment_angle);

    std::vector<Transform> object_transforms;
    float min_y = 42.0f;
    float max_y = 46.0f;

    for(unsigned i = 0; i < 5; i++){
        float cur_y;

        if(i%2 == 0){
            cur_y = (min_y + max_y)/2.0f + i/60.0f * (max_y - min_y);
        }
        else{
            cur_y = (min_y + max_y)/2.0f - i/60.0f * (max_y - min_y);
        }

        Transform new_pose;
        new_pose.mat = rot_mat * Geometry::getMatrixFromTo(Vector3D(0.0f, 0.0f, 1.0f), ramp_normal);
        new_pose.quaternion = Quaternion(new_pose.mat);
        new_pose.shift = start_pos;
        new_pose.shift.x += ramp_xoffset;
        new_pose.shift.y = cur_y;

        object_transforms.push_back(new_pose);
    }

    ArmInverseKinematics ik;
    for(unsigned i = 0; i < object_transforms.size(); i++){
        start_y = object_transforms[i].shift.y;

        Vector3D arm_space_pos = Common::worldPointToArmSpace(object_transforms[i].shift);

        Vector3D axis0 = object_transforms[i].mat * Vector3D(1.0f, 0.0f, 0.0f);
        Vector3D axis1 = object_transforms[i].mat * Vector3D(0.0f, 1.0f, 0.0f);
        Vector3D axis2 = object_transforms[i].mat * Vector3D(0.0f, 0.0f, 1.0f);
        axis0 = Common::worldPointToArmSpace(object_transforms[i].shift + axis0) - arm_space_pos;
        axis1 = Common::worldPointToArmSpace(object_transforms[i].shift + axis1) - arm_space_pos;
        axis2 = Common::worldPointToArmSpace(object_transforms[i].shift + axis2) - arm_space_pos;
        axis0.normalise();
        axis1.normalise();
        axis2.normalise();

        std::vector<Vector3D> basis_vecs;
        basis_vecs.push_back(axis0);
        basis_vecs.push_back(axis1);
        basis_vecs.push_back(axis2);

        Matrix3 arm_space_mat = Geometry::matrixFromBasisVectors(basis_vecs);

        Matrix3 inv_mat;
        inv_mat.isInverse(arm_to_obj_pose.mat);

        arm_space_mat = arm_space_mat*inv_mat;
        arm_space_pos = arm_space_pos - arm_space_mat*arm_to_obj_pose.shift;

        IKResult ik_result = ik.performKDLSearch(arm_space_mat, arm_space_pos);
        if(ik_result.success){          
            while(ik_result.joints[5] > 181.0f){
                ik_result.joints[5] -= 360.0f;
            }
            while(ik_result.joints[5] < -181.0f){
                ik_result.joints[5] += 360.0f;
            }

            error = 0.0f;
            return ik_result.joints;
        }
    }

    error = 1000.0f;
    return std::vector<float>(6, 0.0f);
}
