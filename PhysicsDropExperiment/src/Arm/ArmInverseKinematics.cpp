
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

#define _USE_MATH_DEFINES
#include <math.h>

using namespace KDL;

ArmInverseKinematics::ArmInverseKinematics(){
    initKDL();
}

ArmInverseKinematics::~ArmInverseKinematics(){
    delete iksolver_pos;
    delete iksolver_velocity;
    delete fksolver;
}

IKResult ArmInverseKinematics::getRequiredJoints(OptimisableFunction *target){
	return getRequiredJoints(target, Arm::getArm()->requestCurrentAngles());
}

IKResult ArmInverseKinematics::getRequiredJoints(OptimisableFunction *target, std::vector<float> cur_joints){
    IKResult local_search_result = performLocalSearch(target, cur_joints);
    std::cout << "local search error: "
              << local_search_result.error
              << std::endl;

    if(local_search_result.error > 0.1f){
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

IKResult ArmInverseKinematics::performLocalSearch(OptimisableFunction *target, std::vector<float> cur_joints,unsigned iters){
    IKResult result;
    result.success = true;

    NelderMead optimiser(convertAnglesToParams(cur_joints));

    std::vector<float> result_params = optimiser.optimise(target, 6, iters);
    /*
    target->setDebug(true);
    target->eval(result_params);
    target->setDebug(false);
    */
    ArmForwardKinematics fk;
    result.joints = convertParamsToAngles(result_params);
    result.result_pose = fk.getArmPoint(result.joints);
    result.error = target->eval(result_params);

    return result;
}

IKResult ArmInverseKinematics::performGlobalSearch(OptimisableFunction *target){
    IKResult result;
    result.success = true;

    std::vector<float> cur_joints = Arm::getArm()->requestCurrentAngles();
    RRNelderMead optimiser(3000);
    //ParticleSwarm optimiser(5000);

    std::vector<float> result_params = optimiser.optimise(target, 6, 500);
    target->setDebug(true);
    target->eval(result_params);
    target->setDebug(false);

    ArmForwardKinematics fk;
    result.joints = convertParamsToAngles(result_params);
    result.result_pose = fk.getArmPoint(result.joints);
    result.error = target->eval(result_params);

    return result;
}

IKResult ArmInverseKinematics::performKDLSearch(Matrix3 gripper_pose, Vector3D gripper_pos){
    JntArray q(arm_chain.getNrOfJoints());
    JntArray q_init(arm_chain.getNrOfJoints());

    Rotation rot(gripper_pose(0,0), gripper_pose(0,1), gripper_pose(0,2),
                 gripper_pose(1,0), gripper_pose(1,1), gripper_pose(1,2),
                 gripper_pose(2,0), gripper_pose(2,1), gripper_pose(2,2));

    Frame dest(rot, Vector(gripper_pos.x, gripper_pos.y, gripper_pos.z));
    //std::cout << dest << std::endl;

    int ret = iksolver_pos->CartToJnt(q_init, dest, q);

    IKResult result;
    if(ret < 0){
        result.success = false;
    }
    else{
        result.success = true;

        result.joints.clear();
        for(unsigned i = 0; i < arm_chain.getNrOfJoints(); i++){
            result.joints.push_back(q(i)*180.0f/M_PI);
        }

        ArmForwardKinematics fk;
        result.result_pose = fk.getArmPoint(result.joints);
    }

    return result;
}

void ArmInverseKinematics::testKDL(void){
    JntArray q(arm_chain.getNrOfJoints());
    q(0) = 67.0*M_PI/180.0;
    q(1) = 42.0*M_PI/180.0;
    q(2) = 92.0*M_PI/180.0;
    q(3) = 45.0*M_PI/180.0;
    q(4) = -55.0*M_PI/180.0;
    q(5) = 32.0*M_PI/180.0;

    fksolver->JntToCart(q, cartpos);
    std::cout << cartpos << std::endl;
}

void ArmInverseKinematics::initKDL(void){
    arm_chain.addSegment(Segment(Joint(Joint::None),Frame(Vector(0.0, 0.0, 155.0))));
    arm_chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0, 0.0, 125.0))));
    arm_chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0, 0.0, 210.0))));
    arm_chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(-75.0, 0.0, 90.0))));
    arm_chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0, 0.0, 120.0))));
    arm_chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0, 0.0, 70.0))));
    arm_chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0, 0.0, 180.0))));
    assert(arm_chain.getNrOfJoints() == 6);

    joint_min.resize(arm_chain.getNrOfJoints());
    joint_max.resize(arm_chain.getNrOfJoints());

    joint_min(0) = 0.0*M_PI/180.0;
    joint_min(1) = -40.0*M_PI/180.0;
    joint_min(2) = 20.0*M_PI/180.0;
    joint_min(3) = -159.0*M_PI/180.0;
    joint_min(4) = -114.0*M_PI/180.0;
    joint_min(5) = -350.0*M_PI/180.0;

    joint_max(0) = 110.0*M_PI/180.0;
    joint_max(1) = 85.0*M_PI/180.0;
    joint_max(2) = 159.0*M_PI/180.0;
    joint_max(3) = 159.0*M_PI/180.0;
    joint_max(4) = 114.0*M_PI/180.0;
    joint_max(5) = 350.0*M_PI/180.0;

    fksolver = new KDL::ChainFkSolverPos_recursive(arm_chain);
    iksolver_velocity = new KDL::ChainIkSolverVel_pinv(arm_chain);
    iksolver_pos = new KDL::ChainIkSolverPos_NR_JL(arm_chain, joint_min, joint_max, *fksolver, *iksolver_velocity, 100, 1e-3);
}