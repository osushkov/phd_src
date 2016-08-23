/*
 * IKFitnessFunctionExperimentPickup.h
 *
 *  Created on: 29/09/2010
 *      Author: osushkov
 */

#ifndef IKFITNESSFUNCTIONEXPERIMENTPICKUP_H_
#define IKFITNESSFUNCTIONEXPERIMENTPICKUP_H_

#include "ArmJointLimits.h"
#include "ArmForwardKinematics.h"
#include "../Util/Vector3D.h"
#include "../Util/Quaternion.h"
#include "../OptimiserClass/Optimiser.h"

#include <vector>

class IKFitnessFunctionExperimentPickup : public OptimisableFunction {
  public:
	IKFitnessFunctionExperimentPickup(Vector3D obj_centre,
                                      std::vector<Vector3D> alignment_axes,
                                      std::vector<bool> can_grip_axis);

    float eval(std::vector<float> &params);

  private:
    ArmForwardKinematics forward_kinematics;
    
    Vector3D obj_centre;
    std::vector<Vector3D> alignment_axes;
    std::vector<bool> can_grip_axis;
};

#endif /* IKFITNESSFUNCTIONEXPERIMENTPICKUP_H_ */
