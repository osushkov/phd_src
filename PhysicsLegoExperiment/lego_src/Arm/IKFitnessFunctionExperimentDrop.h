/*
 * IKFitnessFunctionExperimentDrop.h
 *
 *  Created on: 30/09/2010
 *      Author: osushkov
 */

#ifndef IKFITNESSFUNCTIONEXPERIMENTDROP_H_
#define IKFITNESSFUNCTIONEXPERIMENTDROP_H_

#include "ArmJointLimits.h"
#include "ArmForwardKinematics.h"
#include "../Util/Vector3D.h"
#include "../Util/Quaternion.h"
#include "../Util/Transform.h"
#include "../OptimiserClass/Optimiser.h"
#include <vector>

class IKFitnessFunctionExperimentDrop : public OptimisableFunction {
  public:
	IKFitnessFunctionExperimentDrop(Transform gripper_to_object_pose,
									Vector3D experiment_up_vector,
                           	   	 	float experiment_height);

    float eval(std::vector<float> &params);

  private:
    ArmForwardKinematics forward_kinematics;

    Transform gripper_to_object_pose;
    Vector3D experiment_up_vector;
    float experiment_height;
};

#endif /* IKFITNESSFUNCTIONEXPERIMENTDROP_H_ */
