/*
 * ArmForwardKinematics.h
 *
 *  Created on: 21/03/2009
 *      Author: osushkov
 */

#ifndef ARMFORWARDKINEMATICS_H_
#define ARMFORWARDKINEMATICS_H_

#include "../Util/Vector3D.h"
#include <vector>

class ArmForwardKinematics {
  public:
    ArmForwardKinematics();
    ~ArmForwardKinematics();

    std::vector<Vector3D> getArmPoint(std::vector<float> arm_joints);
    std::pair<Vector3D,Vector3D> getGripperPosition(std::vector<float> arm_joints);

    std::vector<float> gripperOrientationToEulerAngles(std::vector<Vector3D> xyz_orientation);
    bool checkJointLimits(std::vector<float> arm_joints);

  private:
    Vector3D findTransforedArmPosition(Vector3D original, std::vector<float> arm_joints);

};

#endif /* ARMFORWARDKINEMATICS_H_ */
