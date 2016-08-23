/*
 * ObjectExperimentTransfer.h
 *
 *  Created on: 30/09/2010
 *      Author: osushkov
 */

#ifndef OBJECTEXPERIMENTTRANSFER_H_
#define OBJECTEXPERIMENTTRANSFER_H_

#include "ObjectGrip.h"
#include "Object.h"
#include "../Util/Transform.h"
#include "../Util/Quaternion.h"

namespace ObjectExperimentTransfer {
	std::vector<float> transferObject(Transform arm_to_obj_pose, 
                                      Vector3D experiment_up_vector, 
                                      float experiment_height,
                                      std::vector<float> cur_joints,
									  float &error);
}


#endif /* OBJECTEXPERIMENTTRANSFER_H_ */
