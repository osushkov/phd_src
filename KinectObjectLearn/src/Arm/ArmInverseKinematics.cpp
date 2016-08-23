
#include "ArmInverseKinematics.h"
#include "ArmForwardKinematics.h"
#include "ArmJointLimits.h"
#include "Arm.h"
#include "IKCommon.h"
#include "../Util/Geometry.h"
#include "../OptimiserClass/Optimiser.h"
#include "../OptimiserClass/RRNelderMead.h"
#include "../OptimiserClass/NelderMead.h"
#include "../OptimiserClass/ParticleSwarm.h"
#include "../OptimiserClass/GeneticAlgorithm.h"


ArmInverseKinematics::ArmInverseKinematics(){

}

ArmInverseKinematics::~ArmInverseKinematics(){

}

IKResult ArmInverseKinematics::getRequiredJoints(OptimisableFunction *target){
    IKResult local_search_result = performLocalSearch(target);
    std::cout << "local search error: "
              << local_search_result.error
              << std::endl;

    if(local_search_result.error > 1.0f){
        IKResult global_search_result = performGlobalSearch(target);
        std::cout << "global search error: "
                  << global_search_result.error
                  << std::endl;

        if(global_search_result.error < local_search_result.error){
            return global_search_result;
        }
    }

    return local_search_result;
}

IKResult ArmInverseKinematics::performLocalSearch(OptimisableFunction *target){
    IKResult result;

    std::vector<float> cur_joints = Arm::getArm()->requestCurrentAngles();
    NelderMead optimiser(convertAnglesToParams(cur_joints));

    std::vector<float> result_params = optimiser.optimise(target, 6, 10000);

    ArmForwardKinematics fk;
    result.joints = convertParamsToAngles(result_params);
    result.result_pose = fk.getArmPoint(result.joints);
    result.error = target->eval(result_params);

    return result;
}

IKResult ArmInverseKinematics::performGlobalSearch(OptimisableFunction *target){
    IKResult result;

    std::vector<float> cur_joints = Arm::getArm()->requestCurrentAngles();
    ParticleSwarm optimiser(500);

    std::vector<float> result_params = optimiser.optimise(target, 6, 1000);

    ArmForwardKinematics fk;
    result.joints = convertParamsToAngles(result_params);
    result.result_pose = fk.getArmPoint(result.joints);
    result.error = target->eval(result_params);

    return result;
}

