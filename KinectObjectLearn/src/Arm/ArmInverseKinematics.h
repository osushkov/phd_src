/*
 * ArmInverseKinematics.h
 *
 *  Created on: 22/03/2009
 *      Author: osushkov
 */

#ifndef ARMINVERSEKINEMATICS_H_
#define ARMINVERSEKINEMATICS_H_

#include "ArmForwardKinematics.h"
#include "IKFitnessFunctionNearest.h"
#include "IKFitnessFunctionObjectPickup.h"
#include "../Util/Vector3D.h"
#include <vector>


struct IKResult {
    std::vector<float> joints;
    std::vector<Vector3D> result_pose;
    float error;
};

class ArmInverseKinematics {
  public:
    ArmInverseKinematics();
    ~ArmInverseKinematics();

    IKResult getRequiredJoints(OptimisableFunction *target);

  private:
    IKResult performLocalSearch(OptimisableFunction *target);
    IKResult performGlobalSearch(OptimisableFunction *target);

};

#endif /* ARMINVERSEKINEMATICS_H_ */
