
#include "ArmForwardKinematics.h"
#include "ArmJointLimits.h"
#include "Geometry.h"

#include <cmath>

ArmForwardKinematics::ArmForwardKinematics(){

}

ArmForwardKinematics::~ArmForwardKinematics(){

}

std::vector<Vector3D> ArmForwardKinematics::getArmPoint(std::vector<float> arm_joints){
    assert(arm_joints.size() == NUM_ARM_JOINTS);

    std::vector<Vector3D> result;
    if(!checkJointLimits(arm_joints)){
        std::cout << "arm joints out of range" << std::endl;
        return result;
    }

    result.push_back(findTransforedArmPosition(Vector3D(0.0f, 0.0f, 0.0f), arm_joints));
    result.push_back(findTransforedArmPosition(Vector3D(1.0f, 0.0f, 0.0f), arm_joints));
    result.push_back(findTransforedArmPosition(Vector3D(0.0f, 1.0f, 0.0f), arm_joints));
    result.push_back(findTransforedArmPosition(Vector3D(0.0f, 0.0f, 1.0f), arm_joints));

    result[1] = result[1] - result[0];
    result[2] = result[2] - result[0];
    result[3] = result[3] - result[0];

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
    xrot *= 180.0f/M_PI;
    yrot *= 180.0f/M_PI;
    zrot *= 180.0f/M_PI;

    std::vector<float> result;
    result.push_back(xrot);
    result.push_back(yrot);
    result.push_back(zrot);

    return result;
}

Vector3D ArmForwardKinematics::findTransforedArmPosition(Vector3D original,
                                                         std::vector<float> arm_joints){
    assert(arm_joints.size() == NUM_ARM_JOINTS);
    std::vector<Vector3D> rot_mat;

    rot_mat = Geometry::axisRotationMatrix(Vector3D(0.0f, 0.0f, 1.0f), arm_joints[5]*M_PI/180.0f);
    original.matrixMultLeft(rot_mat);
    original = original + Vector3D(0.0f, 0.0f, 7.0f);

    rot_mat = Geometry::axisRotationMatrix(Vector3D(0.0f, 1.0f, 0.0f), arm_joints[4]*M_PI/180.0f);
    original.matrixMultLeft(rot_mat);
    original = original + Vector3D(0.0f, 0.0f, 12.0f);

    rot_mat = Geometry::axisRotationMatrix(Vector3D(0.0f, 0.0f, 1.0f), arm_joints[3]*M_PI/180.0f);
    original.matrixMultLeft(rot_mat);
    original = original + Vector3D(-7.5f, 0.0f, 9.0f);

    rot_mat = Geometry::axisRotationMatrix(Vector3D(0.0f, 1.0f, 0.0f), arm_joints[2]*M_PI/180.0f);
    original.matrixMultLeft(rot_mat);
    original = original + Vector3D(0.0f, 0.0f, 21.0f);

    rot_mat = Geometry::axisRotationMatrix(Vector3D(0.0f, 1.0f, 0.0f), arm_joints[1]*M_PI/180.0f);
    original.matrixMultLeft(rot_mat);
    original = original + Vector3D(0.0f, 0.0f, 12.5f);

    rot_mat = Geometry::axisRotationMatrix(Vector3D(0.0f, 0.0f, 1.0f), arm_joints[0]*M_PI/180.0f);
    original.matrixMultLeft(rot_mat);
    original = original + Vector3D(0.0f, 0.0f, 15.5f);

    return original;
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

