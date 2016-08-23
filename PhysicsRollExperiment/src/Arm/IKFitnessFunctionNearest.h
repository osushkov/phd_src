/*
 * IKFitnessFunctionNearest.h
 *
 *  Created on: 26/03/2009
 *      Author: osushkov
 */

#ifndef IKFITNESSFUNCTIONNEAREST_H_
#define IKFITNESSFUNCTIONNEAREST_H_

#include "ArmJointLimits.h"
#include "ArmForwardKinematics.h"
#include "../Util/Vector3D.h"
#include "../OptimiserClass/Optimiser.h"

#include <vector>

class IKFitnessFunctionNearest : public OptimisableFunction {
  public:
    IKFitnessFunctionNearest(std::vector<Vector3D> target_pose);

    float eval(std::vector<float> &params);

  private:
    ArmForwardKinematics forward_kinematics;
    std::vector<Vector3D> target_pose;
};

#endif /* IKFITNESSFUNCTIONNEAREST_H_ */
