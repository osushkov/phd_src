/*
 * ArmJointLimits.h
 *
 *  Created on: 22/03/2009
 *      Author: osushkov
 */

#ifndef ARMJOINTLIMITS_H_
#define ARMJOINTLIMITS_H_

#define NUM_ARM_JOINTS 6

const float ArmJointMinLimits[] = { -0.0f, -40.0f, 20.0f, -159.0f, -114.0f, -200.0f };
const float ArmJointMaxLimits[] = {  110.0f,  85.0f, 159.0f, 159.0f,  114.0f,  200.0f };

#endif /* ARMJOINTLIMITS_H_ */
