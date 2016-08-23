
#include "ArmForwardKinematics.h"
#include "ArmJointLimits.h"
#include "../Util/Geometry.h"

#define _USE_MATH_DEFINES
#include <math.h>

ArmForwardKinematics::ArmForwardKinematics(){

}

ArmForwardKinematics::~ArmForwardKinematics(){

}

std::vector<Vector3D> ArmForwardKinematics::getArmPoint(std::vector<float> arm_joints){
    assert(arm_joints.size() == NUM_ARM_JOINTS);

    std::vector<Vector3D> result;
    /*if(!checkJointLimits(arm_joints)){
        std::cout << "arm joints out of range" << std::endl;
        return result;
    }*/

    result.push_back(findTransforedArmPosition(Vector3D(0.0f, 0.0f, 0.0f), arm_joints));
    result.push_back(findTransforedArmPosition(Vector3D(1.0f, 0.0f, 0.0f), arm_joints));
    result.push_back(findTransforedArmPosition(Vector3D(0.0f, 1.0f, 0.0f), arm_joints));
    result.push_back(findTransforedArmPosition(Vector3D(0.0f, 0.0f, 1.0f), arm_joints));

    result[1] = result[1] - result[0];
    result[2] = result[2] - result[0];
    result[3] = result[3] - result[0];

    result[1].normalise();
    result[2].normalise();
    result[3].normalise();

    return result;
}

std::pair<Vector3D,Vector3D> ArmForwardKinematics::getGripperPosition(std::vector<float> arm_joints){
	std::vector<Vector3D> arm_pose = getArmPoint(arm_joints);
	Matrix3 rot_mat = Geometry::axisRotationMatrix(arm_pose[3], -10.0f*(float)M_PI/180.0f);
	Vector3D offset_vec = arm_pose[1];
	offset_vec = rot_mat * offset_vec;

	std::pair<Vector3D,Vector3D> result;
	result.first = arm_pose[0] + 60.0f*offset_vec;
	result.second = arm_pose[0] - 60.0f*offset_vec;
	return result;
}

std::vector<float>
ArmForwardKinematics::gripperOrientationToEulerAngles(std::vector<Vector3D> xyz_orientation){
    assert(xyz_orientation.size() == 3);

    Vector3D gripper_x = xyz_orientation[0];
    Vector3D gripper_y = xyz_orientation[1];
    Vector3D gripper_z = xyz_orientation[2];

    float yrot = asin(-gripper_x.z);
    float xrot = atan2(gripper_y.z, gripper_z.z);
    float zrot = atan2(gripper_x.y, gripper_x.x);

    // Convert to degrees
    xrot *= 180.0f/(float)M_PI;
    yrot *= 180.0f/(float)M_PI;
    zrot *= 180.0f/(float)M_PI;

    std::vector<float> result;
    result.push_back(xrot);
    result.push_back(yrot);
    result.push_back(zrot);

    return result;
}

bool ArmForwardKinematics::checkJointLimits(std::vector<float> arm_joints){
    assert(arm_joints.size() == NUM_ARM_JOINTS);
    for(unsigned i = 0; i < arm_joints.size(); i++){
        if(arm_joints[i] < ArmJointMinLimits[i] || arm_joints[i] > ArmJointMaxLimits[i]){
            return false;
        }
    }

    return true;
}

Vector3D ArmForwardKinematics::findTransforedArmPosition(Vector3D original,
                                                         std::vector<float> arm_joints){
    assert(arm_joints.size() == NUM_ARM_JOINTS);
    Matrix3 rot_mat;

    original = original + Vector3D(0.0f, 0.0f, 180.0f);

    rot_mat = Geometry::axisRotationMatrix(Vector3D(0.0f, 0.0f, 1.0f), arm_joints[5]*(float)M_PI/180.0f);
    original = rot_mat * original;
    original = original + Vector3D(0.0f, 0.0f, 70.0f);

    rot_mat = Geometry::axisRotationMatrix(Vector3D(0.0f, 1.0f, 0.0f), arm_joints[4]*(float)M_PI/180.0f);
    original = rot_mat * original;
    original = original + Vector3D(0.0f, 0.0f, 120.0f);

    rot_mat = Geometry::axisRotationMatrix(Vector3D(0.0f, 0.0f, 1.0f), arm_joints[3]*(float)M_PI/180.0f);
    original = rot_mat * original;
    original = original + Vector3D(-75.0f, 0.0f, 90.0f);

    rot_mat = Geometry::axisRotationMatrix(Vector3D(0.0f, 1.0f, 0.0f), arm_joints[2]*(float)M_PI/180.0f);
    original = rot_mat * original;
    original = original + Vector3D(0.0f, 0.0f, 210.0f);

    rot_mat = Geometry::axisRotationMatrix(Vector3D(0.0f, 1.0f, 0.0f), arm_joints[1]*(float)M_PI/180.0f);
    original = rot_mat * original;
    original = original + Vector3D(0.0f, 0.0f, 125.0f);

    rot_mat = Geometry::axisRotationMatrix(Vector3D(0.0f, 0.0f, 1.0f), arm_joints[0]*(float)M_PI/180.0f);
    original = rot_mat * original;
    original = original + Vector3D(0.0f, 0.0f, 155.0f);

    return original;
}

