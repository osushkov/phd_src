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
															Vector3D experiment_up_vector, 
                                                            float experiment_height,
                                                            std::vector<float> cur_joints,
															float &error){

    SuperQuadric shape0(0.1f, 0.1f, 2.75f, 4.2f, 5.95f);
    std::vector<Vector3D> surface_points = shape0.getSurfacePoints(M_PI*0.05f);

    Matrix3 orientation_mat = Geometry::getMatrixFromTo(experiment_up_vector, Vector3D(0.0f, 0.0f, 1.0f));

    float lowest_point = 0.0f;
    for(unsigned i = 0; i < surface_points.size(); i++){
        Vector3D tpoint = orientation_mat*surface_points[i];
        if(i == 0 || tpoint.z < lowest_point){
            lowest_point = tpoint.z;
        }
    }

    float final_height = experiment_height - lowest_point + 11.9f;
    std::cout << "experiment height: " << final_height << " " << experiment_height << " " << lowest_point << std::endl;

    std::vector<std::pair<Vector3D, Matrix3> > possible_drop_poses = 
        getPossibleDropPoses(arm_to_obj_pose, experiment_up_vector, final_height);

    ArmInverseKinematics ik;
    for(unsigned i = 0; i < possible_drop_poses.size(); i++){
        IKResult ik_result = ik.performKDLSearch(possible_drop_poses[i].second, possible_drop_poses[i].first);
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
