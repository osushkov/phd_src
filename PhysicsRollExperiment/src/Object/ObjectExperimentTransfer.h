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
                                      float box_bottom_offset,
                                      float experiment_height, 
                                      float experiment_angle,
                                      float ramp_xoffset,
                                      Vector3D ramp_normal,
                                      std::vector<float> cur_joints,
									  float &error,
                                      float &start_y);
}


#endif /* OBJECTEXPERIMENTTRANSFER_H_ */
