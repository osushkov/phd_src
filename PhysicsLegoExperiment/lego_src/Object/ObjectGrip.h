/*
 * ObjectGrip.h
 *
 *  Created on: 28/09/2010
 *      Author: osushkov
 */

#ifndef OBJECTGRIP_H_
#define OBJECTGRIP_H_

#include "Object.h"
#include "../Util/Vector3D.h"
#include "../Util/Transform.h"
#include <vector>

namespace ObjectGrip {

	struct GripData {
		bool success;
		std::vector<float> root_joints;
		std::vector<float> arm_joints;
		Vector3D grip_point;
	};

	GripData gripObject(Object *object, Transform obj_pose);

	Transform findObjectPose(std::vector<Vector3D> gripper_pose, Transform arm_to_obj_pose);
    Transform armToObjectPoseTransform(Transform obj_pose, std::vector<float> grip_arm_joints);

}

#endif /* OBJECTGRIP_H_ */
