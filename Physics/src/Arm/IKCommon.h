/*
 * IKCommon.h
 *
 *  Created on: 26/03/2009
 *      Author: osushkov
 */

#ifndef IKCOMMON_H_
#define IKCOMMON_H_

inline std::vector<float> convertParamsToAngles(const std::vector<float> &params) {
    assert(params.size() == NUM_ARM_JOINTS);

    std::vector<float> cur_joint_angles;
    for (unsigned i = 0; i < params.size(); i++) {
        float joint_angle =
            ArmJointMinLimits[i] + params[i]*(ArmJointMaxLimits[i]-ArmJointMinLimits[i]);

        cur_joint_angles.push_back(joint_angle);
    }
    return cur_joint_angles;
}

inline std::vector<float> convertAnglesToParams(const std::vector<float> &angles){
    std::vector<float> result;
    for (unsigned i = 0; i < angles.size(); i++) {
        float joint_range = ArmJointMaxLimits[i] - ArmJointMinLimits[i];
        result.push_back((angles[i] - ArmJointMinLimits[i])/joint_range);
    }
    return result;
}

inline float calculateJointStress(const std::vector<float> &params){
    assert(params.size() == NUM_ARM_JOINTS);

    float result = 0.0f;
    for(unsigned i = 0; i < params.size(); i++){
        if(fabs(params[i]) > 0.9f){
            result += (params[i] - 0.5f)*(params[i] - 0.5f);
        }
    }

    return result;
}


#endif /* IKCOMMON_H_ */
