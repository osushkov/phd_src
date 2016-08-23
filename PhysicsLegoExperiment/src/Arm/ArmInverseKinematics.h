/*
 * ArmInverseKinematics.h
 *
 *  Created on: 22/03/2009
 *      Author: osushkov
 */

#ifndef ARMINVERSEKINEMATICS_H_
#define ARMINVERSEKINEMATICS_H_

#include "ArmForwardKinematics.h"
#include "IKFitnessFunctionNearest.h"
#include "IKFitnessFunctionObjectPickup.h"
#include "../Util/Vector3D.h"
#include <vector>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames_io.hpp>


struct IKResult {
    std::vector<float> joints;
    std::vector<Vector3D> result_pose;
    float error;
    bool success;
};

class ArmInverseKinematics {
  public:
    ArmInverseKinematics();
    ~ArmInverseKinematics();

    IKResult getRequiredJoints(OptimisableFunction *target);
    IKResult getRequiredJoints(OptimisableFunction *target, std::vector<float> cur_joints);

    IKResult performLocalSearch(OptimisableFunction *target, std::vector<float> cur_joints, unsigned iters=1000);
    IKResult performGlobalSearch(OptimisableFunction *target);

    IKResult performKDLSearch(Matrix3 gripper_pose, Vector3D gripper_pos);

    void testKDL(void);

  private:
    KDL::Chain arm_chain;
    KDL::JntArray joint_min, joint_max;

    KDL::ChainFkSolverPos_recursive *fksolver;
    KDL::ChainIkSolverVel_pinv *iksolver_velocity;
    KDL::ChainIkSolverPos_NR_JL *iksolver_pos;

    KDL::Frame cartpos;

    void initKDL(void);

};

#endif /* ARMINVERSEKINEMATICS_H_ */
