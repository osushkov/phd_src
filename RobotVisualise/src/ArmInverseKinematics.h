/*
 * ArmInverseKinematics.h
 *
 *  Created on: 22/03/2009
 *      Author: osushkov
 */

#ifndef ARMINVERSEKINEMATICS_H_
#define ARMINVERSEKINEMATICS_H_

#include "ArmForwardKinematics.h"
#include "Vector3D.h"
#include <vector>

class ArmInverseKinematics {
  public:
    ArmInverseKinematics();
    ~ArmInverseKinematics();

    std::vector<float> getRequiredJoints(Vector3D position);

  private:


};

#endif /* ARMINVERSEKINEMATICS_H_ */
