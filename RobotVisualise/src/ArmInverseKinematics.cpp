
#include "ArmInverseKinematics.h"
#include "ArmForwardKinematics.h"
#include "ArmJointLimits.h"
#include "OptimiserClass/Optimiser.h"
#include "OptimiserClass/NelderMead.h"

static std::vector<float> convertParamsToAngles(const std::vector<float> &params) {
    assert(params.size() == NUM_ARM_JOINTS);

    std::vector<float> cur_joint_angles;
    for (unsigned i = 0; i < params.size(); i++) {
        float joint_angle =
            ArmJointMinLimits[i] + params[i] * (ArmJointMaxLimits[i] - ArmJointMinLimits[i]);

        cur_joint_angles.push_back(joint_angle);
    }
    return cur_joint_angles;
}

class IKOptimiser : public OptimisableFunction {
  public:
    IKOptimiser(Vector3D target_position) :
        target_position(target_position){

    }

    float eval(std::vector<float> &params){
        assert(params.size() == NUM_ARM_JOINTS);

        std::vector<float> cur_joint_angles = convertParamsToAngles(params);
        Vector3D generated_position = forward_kinematics.getArmPoint(cur_joint_angles).at(0);
        return (target_position - generated_position).length();
    }

  private:
    ArmForwardKinematics forward_kinematics;
    Vector3D target_position;

};


ArmInverseKinematics::ArmInverseKinematics(){

}
ArmInverseKinematics::~ArmInverseKinematics(){

}

std::vector<float> ArmInverseKinematics::getRequiredJoints(Vector3D position){
    std::vector<float> result;

    IKOptimiser *target_function = new IKOptimiser(position);
    NelderMead optimiser;
    result = optimiser.optimise(target_function, 6, 1000, std::vector<float>());

    float dist = target_function->eval(result);
    std::cout << "best dist: " << dist << std::endl;

    result = convertParamsToAngles(result);

    delete target_function;
    return result;
}
