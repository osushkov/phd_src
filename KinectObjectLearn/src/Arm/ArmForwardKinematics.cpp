
#include "ArmForwardKinematics.h"
#include "ArmJointLimits.h"
#include "../Util/Geometry.h"
#include "../Util/Common.h"


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

    result.push_back(findTransformedArmPosition(Vector3D(0.0f, 0.0f, 0.0f), arm_joints));
    result.push_back(findTransformedArmPosition(Vector3D(1.0f, 0.0f, 0.0f), arm_joints));
    result.push_back(findTransformedArmPosition(Vector3D(0.0f, 1.0f, 0.0f), arm_joints));
    result.push_back(findTransformedArmPosition(Vector3D(0.0f, 0.0f, 1.0f), arm_joints));

    result[1] = result[1] - result[0];
    result[2] = result[2] - result[0];
    result[3] = result[3] - result[0];

    result[1].normalise();
    result[2].normalise();
    result[3].normalise();

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

bool ArmForwardKinematics::checkJointLimits(std::vector<float> arm_joints){
    assert(arm_joints.size() == NUM_ARM_JOINTS);
    for(unsigned i = 0; i < arm_joints.size(); i++){
        if(arm_joints[i] < ArmJointMinLimits[i] || arm_joints[i] > ArmJointMaxLimits[i]){
            return false;
        }
    }

    return true;
}

Vector3D ArmForwardKinematics::getToCameraVector(std::vector<float> arm_joints){
    Vector3D camera_world_pos = Common::cameraPointToWorldSpace(Vector3D(0.0f, 0.0f, 0.0f), 0.0f, 25.0f);
    std::vector<Vector3D> arm_pose = getArmPoint(arm_joints);
    std::vector<Vector3D> arm_orientation;
    arm_orientation.push_back(arm_pose[1]);
    arm_orientation.push_back(arm_pose[2]);
    arm_orientation.push_back(arm_pose[3]);

    Vector3D to_camera = camera_world_pos - Common::armPointToWorldSpace(arm_pose[0]);
    to_camera.normalise();

    Matrix3 orientation = Geometry::matrixFromBasisVectors(arm_orientation);
    Matrix3 inv_ori;
    inv_ori.isInverse(orientation);

    return inv_ori*to_camera;
}

Vector3D ArmForwardKinematics::findTransformedArmPosition(Vector3D original,
                                                         std::vector<float> arm_joints){
    assert(arm_joints.size() == NUM_ARM_JOINTS);
    std::vector<Vector3D> rot_mat;

    original = original + Vector3D(0.0f, 0.0f, 18.0f);

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

