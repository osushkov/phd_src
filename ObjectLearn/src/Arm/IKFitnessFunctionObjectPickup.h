/*
 * IKFitnessFunctionObjectPickup.h
 *
 *  Created on: 26/03/2009
 *      Author: osushkov
 */

#ifndef IKFITNESSFUNCTIONOBJECTPICKUP_H_
#define IKFITNESSFUNCTIONOBJECTPICKUP_H_

#include "ArmJointLimits.h"
#include "ArmForwardKinematics.h"
#include "../Util/Vector3D.h"
#include "../OptimiserClass/Optimiser.h"

#include <vector>

class IKFitnessFunctionObjectPickup : public OptimisableFunction {
  public:
    IKFitnessFunctionObjectPickup(Vector3D object_position,
                                  std::vector<float> cur_joints);

    float eval(std::vector<float> &params, float iter_fraction);

  private:
    ArmForwardKinematics forward_kinematics;
    Vector3D object_position;
    std::vector<float> cur_joints;
};

#endif /* IKFITNESSFUNCTIONOBJECTPICKUP_H_ */
