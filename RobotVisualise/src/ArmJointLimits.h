/*
 * ArmJointLimits.h
 *
 *  Created on: 22/03/2009
 *      Author: osushkov
 */

#ifndef ARMJOINTLIMITS_H_
#define ARMJOINTLIMITS_H_

#define NUM_ARM_JOINTS 6

const float ArmJointMinLimits[] = { -160.0f, -120.0f, 19.0f, -160.0f, -120.0f, -360.0f };
const float ArmJointMaxLimits[] = {  160.0f,  120.0f, 160.0f, 160.0f,  120.0f,  360.0f };

#endif /* ARMJOINTLIMITS_H_ */
