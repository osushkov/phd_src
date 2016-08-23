/*
 * ArmForwardKinematics.h
 *
 *  Created on: 21/03/2009
 *      Author: osushkov
 */

#ifndef ARMFORWARDKINEMATICS_H_
#define ARMFORWARDKINEMATICS_H_

#include "Vector3D.h"
#include <vector>

class ArmForwardKinematics {
  public:
    ArmForwardKinematics();
    ~ArmForwardKinematics();

    std::vector<Vector3D> getArmPoint(std::vector<float> arm_joints);
    std::vector<float> gripperOrientationToEulerAngles(std::vector<Vector3D> xyz_orientation);

  private:
    Vector3D findTransforedArmPosition(Vector3D original, std::vector<float> arm_joints);
    bool checkJointLimits(std::vector<float> arm_joints);
};

#endif /* ARMFORWARDKINEMATICS_H_ */
