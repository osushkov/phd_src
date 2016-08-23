
#include "Control.h"

#include <vector>
#include <iostream>
#include <list>
#include <pthread.h>
#include <cstdio>

#define _USE_MATH_DEFINES
#include <math.h>

#include "Arm/Arm.h"
#include "Arm/ArmInverseKinematics.h"
#include "Arm/IKFitnessFunctionNearest.h"
#include "Arm/IKFitnessFunctionObjectPickup.h"
#include "Visualisation/SceneRenderer.h"
#include "Visualisation/PointCloudRenderObject.h"
#include "Visualisation/SuperQuadricRenderObject.h"
#include "Visualisation/BoxRenderObject.h"
#include "Visualisation/MeshRenderObject.h"
#include "Visualisation/ArmRenderObject.h"
#include "Settings.h"
#include "CameraController.h"
#include "Util/Common.h"
#include "Util/Geometry.h"
#include "Object/ObjectGrip.h"
#include "Object/ObjectExperimentTransfer.h"

static std::vector<std::vector<float> > root_arm_poses;

static void setRootArmPoses(void){
	float arm_poses[8][6] = {
			{30.31f, 48.04f, 83.4f, -71.38f, -67.87f, 155.77f},
			{31.95f, 91.45f, 70.46f, -61.48f, -79.51f, 126.31f},
			{58.43f, 4.06f, 131.66f, -43.49f, -52.94f, 139.19f},
			{73.49f, 24.98f, 87.75f, -59.32f, 30.16f, 166.59f},
			{52.77f, 52.63f, 72.12f, -120.93f, 10.55f, 219.31f},
			{63.94f, 90.74f, 62.88f, -154.84f, 104.68f, 209.15f},
			{78.47f, 63.74f, 92.49f, -141.39f, 111.07f, 202.37f},
			{43.52f, 62.23f, 41.68f, 64.63f, 76.68f, 109.41f}
	};

	for(unsigned i = 0; i < 8; i++){
		std::vector<float> new_pose;
		for(unsigned j = 0; j < 6; j++){
			new_pose.push_back(arm_poses[i][j]);
		}
		root_arm_poses.push_back(new_pose);
	}
}

std::vector<std::vector<float> > Control::getArmRootPoses(void){
	if(root_arm_poses.size() == 0){
		setRootArmPoses();
	}
	return root_arm_poses;
}


Transform Control::locateObject(Object *obj, unsigned max_frames, unsigned &num_features_matched, bool &success){
    if(obj->isTransformValid()){
        success = true;
        return obj->getObjectTransform();
    }

    std::cout << "Control: locateObject" << std::endl;
    SceneRenderer::instance().initialise();

    PointCloudRenderObject *point_cloud = new PointCloudRenderObject(false, Vector3D(0.0f,1.0f,0.0f));
    SceneRenderer::instance().addObject(point_cloud);

    CameraController camera_controller(true, "", true, true);
    camera_controller.createViewWindows();

    Transform result;
    result.shift = Vector3D(0.0f, 0.0f, 0.0f);
    result.quaternion = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);

    num_features_matched = 0;
    unsigned num_detected_frames = 0;
    unsigned frame = 0;
    obj->resetSceneMatch();
    while (camera_controller.isActive() && frame < max_frames) {
        if (!camera_controller.getNewFrame()) {
            break;
        }

        std::vector<StereoFeature> scene_features = camera_controller.getStereoFeatures();
        for(unsigned i = 0; i < scene_features.size(); i++){
            scene_features[i].position =
                Common::cameraPointToWorldSpace(scene_features[i].position, 0.0f, -63.0f, 0.0f);
        }

        filterWorkspaceFeatures(scene_features);

        std::vector<PointCloudPoint> point_cloud_points;
        for(unsigned i = 0; i < scene_features.size(); i++){
            PointCloudPoint new_point;
            new_point.pos = scene_features[i].position;
            point_cloud_points.push_back(new_point);
        }
        point_cloud->clearPoints();
        point_cloud->addPoints(point_cloud_points);


        Transform object_transform;
        unsigned cur_num_features_matched;
        if(obj->sceneMatch(scene_features, object_transform, cur_num_features_matched)){
        	obj->render(object_transform);

            result.shift = object_transform.shift;
            result.quaternion = object_transform.quaternion;

            num_detected_frames++;
            num_features_matched += cur_num_features_matched;
        }
        else{
        	std::cout << "Could not match scene object" << std::endl;
        }

        if(camera_controller.showCameraViewAndContinue() == 'q'){
            break;
        }

        frame++;
    }
/*
    result.shift.scale(1.0f/num_detected_frames);
    result.quaternion.normalise(); //= Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
*/ 
    result.mat = result.quaternion.toMatrix();
    result.secondary_shift = Vector3D(0.0f, 0.0f, 0.0f);

    SceneRenderer::instance().clearObjects();
    delete point_cloud;

    std::cout << "Located Object:" << std::endl;
    result.mat.printOut();
    result.shift.print();

    std::cout << "Finished locateObject" << std::endl;
    obj->setObjectTransform(result);

    success = num_detected_frames > 0;
    return result;
}


static void displayGripPath(ObjectGrip::GripData grip, Object *object, Transform obj_pose){
	SceneRenderer::instance().clearObjects();
	object->render(obj_pose);

	ArmRenderObject *arm_ro = new ArmRenderObject();
	arm_ro->setJointAngles(grip.root_joints);
	SceneRenderer::instance().addObject(arm_ro);

	arm_ro->moveTo(grip.arm_joints, 2.0f);
}

static void displayGripPath(std::vector<float> src_joints, std::vector<float> dst_joints, Object *object, Transform arm_to_obj_pose){
	SceneRenderer::instance().clearObjects();

	ArmRenderObject *arm_ro = new ArmRenderObject();
	SceneRenderer::instance().addObject(arm_ro);

	const unsigned frames = 100;
	unsigned ms_pause = 2*1000/frames;

	ArmForwardKinematics forward_kinematics;

	for(unsigned i = 0; i < frames; i++){
		std::vector<float> joint_angles;
		for(unsigned j = 0; j < src_joints.size(); j++){
			joint_angles.push_back((float)i/(float)frames*dst_joints[j] + (1.0f - (float)i/(float)frames)*src_joints[j]);;
		}
		arm_ro->setJointAngles(joint_angles);

		std::vector<Vector3D> gripper_pose = forward_kinematics.getArmPoint(joint_angles);
		Transform object_pose = ObjectGrip::findObjectPose(gripper_pose, arm_to_obj_pose);
		object->render(object_pose);
/*
		Transform t;
		t.shift = Common::armPointToWorldSpace(gripper_pose[0]);
		t.mat.identity();
		sq_ro->setTransform(t);
*/

        Common::sleep(ms_pause);
	}
}

static float calculateGripSize(ObjectGrip::GripData grip_data, Object *obj, Transform cur_pose){
    ArmForwardKinematics fk;

    Matrix3 inv_pose = cur_pose.mat;
    inv_pose.isInverse(cur_pose.mat);

    std::pair<Vector3D,Vector3D> gripper_points = fk.getGripperPosition(grip_data.arm_joints);

    gripper_points.first = Common::armPointToWorldSpace(gripper_points.first);
    gripper_points.first = gripper_points.first - cur_pose.shift;
    gripper_points.first = inv_pose * gripper_points.first;
    
    gripper_points.second = Common::armPointToWorldSpace(gripper_points.second);
    gripper_points.second = gripper_points.second - cur_pose.shift;
    gripper_points.second = inv_pose * gripper_points.second;

    return obj->findGripSize(gripper_points.first, gripper_points.second);
}

static ObjectGrip::GripData last_grip_data;

Transform Control::pickUpObject(Object *obj, bool &success){
    assert(obj != NULL);

    Arm::getArm()->releaseHand();
    Control::moveArmOutOfTheWay();

    unsigned num_features_detected = 0;
    bool can_locate_object = false;
    Transform cur_pose = locateObject(obj, 1, num_features_detected, can_locate_object);

    if(!can_locate_object){
        success = false;
        return cur_pose;
    }

    ObjectGrip::GripData grip_data = ObjectGrip::gripObject(obj, cur_pose);
    if(!grip_data.success){
        success = false;
        return cur_pose;
    }

    last_grip_data = grip_data;
    Transform arm_to_object = ObjectGrip::armToObjectPoseTransform(cur_pose, grip_data.arm_joints);
    float grip_size = 8.5f;//calculateGripSize(grip_data, obj, cur_pose);

    displayGripPath(grip_data, obj, cur_pose);
    std::cout << "move?" << std::endl;
    getchar();

    obj->invalidateTransform();
    Control::safeMoveToJoints(grip_data.arm_joints);
    Common::sleep(1500);
    Arm::getArm()->closeHandDistance(grip_size-0.8f);
    Common::sleep(2000);

    success = true; 
    return arm_to_object;
}

Transform Control::refineArmToObject(Object *obj){
    std::cout << "Refining Arm to Object Transform" << std::endl;

    unsigned best_num_features_detected = 0;
    Transform best_pose;
    std::vector<float> best_joint_angles;

    for(unsigned i = 0; i < 2; i++){
        obj->invalidateTransform();
        Control::moveToObjectExamineArmPose(i);

        unsigned num_features_detected = 0;
        bool success = false;
        Transform cur_pose = locateObject(obj, 1, num_features_detected, success);

        if(num_features_detected > best_num_features_detected){
            best_num_features_detected = num_features_detected;
            best_pose = cur_pose;
            best_joint_angles = Arm::getArm()->requestCurrentAngles();
        }
    }
    return ObjectGrip::armToObjectPoseTransform(best_pose, best_joint_angles);
}

static std::vector<std::pair<Vector3D, Matrix3> > getPossibleArmPoses(Vector3D light_world_pos){
    std::vector<std::pair<Vector3D, Matrix3> > result;

    Vector3D gripper_pos = Common::worldPointToArmSpace(light_world_pos);

    Vector3D x_axis(1.0f, 0.0f, 0.0f);
    Vector3D y_axis(0.0f, 1.0f, 0.0f);
    Vector3D z_axis(0.0f, 0.0f, 1.0f);

    std::vector<Vector3D> basis_set;
    basis_set.push_back(x_axis);
    basis_set.push_back(y_axis);
    basis_set.push_back(z_axis);

    Matrix3 ori_mat = Geometry::matrixFromBasisVectors(basis_set);
    Matrix3 gripper_rot_mat = Geometry::axisRotationMatrix(z_axis, 7.0f * M_PI/180.0f);
    ori_mat = gripper_rot_mat*ori_mat;

    const unsigned num_angles = 60;
    for(unsigned i = 0; i < num_angles; i++){
        Matrix3 rot_mat = Geometry::axisRotationMatrix(y_axis, (float)i/(float)num_angles * 2.0f * M_PI);

        result.push_back(std::pair<Vector3D,Matrix3>(gripper_pos, rot_mat*ori_mat));
    }

    return result;
}

bool Control::moveToExperiment(Object *obj, Transform object_pose, float floor_height,
                               LightExperiment *exp, float error_threshold){
    
    Vector3D light_world_position = object_pose.mat * exp->getLightPos() + object_pose.shift;

    const float light_angle = 7.12f * M_PI/180.0f;
    float light_height = floor_height + exp->getLightRadius()/tanf(light_angle) + 6.0f;
    light_world_position.z = light_height;

    std::vector<std::pair<Vector3D, Matrix3> > possible_arm_poses = getPossibleArmPoses(light_world_position);

    bool can_perform = false;
    ArmInverseKinematics ik;
    for(unsigned i = 0; i < possible_arm_poses.size(); i++){
        IKResult ik_result = ik.performKDLSearch(possible_arm_poses[i].second, possible_arm_poses[i].first);
        if(ik_result.success){
            Control::safeMoveToJoints(ik_result.joints);
            can_perform = true;
            break;
        }
    }

    return can_perform;

    /*
    float error;

    std::vector<float> experiment_joints = ObjectExperimentTransfer::transferObject(arm_to_object, box_bottom_offset, 
        exp->getDropHeight(), exp->getObjRotation(), 0.0f, ramp_normal, last_grip_data.arm_joints, error, start_y);

    ArmForwardKinematics fk;
    std::pair<Vector3D,Vector3D> gripper_pos = fk.getGripperPosition(experiment_joints);

    gripper_pos.first = Common::armPointToWorldSpace(gripper_pos.first);
    gripper_pos.second = Common::armPointToWorldSpace(gripper_pos.second);
    
    Vector3D middle = 0.5f*(gripper_pos.first + gripper_pos.second);
    
    Vector3D to_gripper0 = gripper_pos.first - middle;
    to_gripper0.normalise();

    Vector3D to_gripper1 = gripper_pos.second - middle;
    to_gripper1.normalise();

    if(error < error_threshold){
        displayGripPath(last_grip_data.arm_joints, experiment_joints, obj, arm_to_object);
        obj->invalidateTransform();
        Control::safeMoveToJoints(experiment_joints);
        return true;
    }
    else{
        return false;
    }
    */
    return true;
}

void Control::moveToObjectExamineArmPose(unsigned pose_index){
    std::vector<float> examine_joints;
    examine_joints.push_back(67.35f);
    examine_joints.push_back(48.08f);
    examine_joints.push_back(55.55f);
    examine_joints.push_back(133.64f);
    examine_joints.push_back(-65.01f);
    examine_joints.push_back(110.0f);

    if(pose_index == 1){
        examine_joints[5] -= 180.0f;
    }
    else{
        std::cerr << "Incorrect pose index: " << pose_index << std::endl;
    }

    Control::safeMoveToJoints(examine_joints);
    Common::sleep(1000);
}

void Control::moveArmToJoints(std::vector<float> joints){
    ArmForwardKinematics afk;
    std::vector<Vector3D> arm_pose = afk.getArmPoint(joints);

    std::vector<Vector3D> gripper_pose;
    gripper_pose.push_back(arm_pose[1]);
    gripper_pose.push_back(arm_pose[2]);
    gripper_pose.push_back(arm_pose[3]);

    std::vector<float> gripper_euler_angles = afk.gripperOrientationToEulerAngles(gripper_pose);

    Arm *arm = Arm::getArm();
    arm->moveToPoint(arm_pose[0].x, arm_pose[0].y, arm_pose[0].z,
    				 gripper_euler_angles[0],
    				 gripper_euler_angles[1],
    				 gripper_euler_angles[2]);
}

std::vector<float> gripperOrientationToEulerAngles(std::vector<Vector3D> xyz_orientation){
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

std::vector<Vector3D> Control::gripperEulerAnglesToOrientation(std::vector<float> euler_angles){
    Matrix3 rot_mat1 = Geometry::axisRotationMatrix(Vector3D(0.0f, 1.0f, 0.0f), euler_angles[1]*(float)M_PI/180.0f);
    Matrix3 rot_mat2 = Geometry::axisRotationMatrix(Vector3D(1.0f, 0.0f, 0.0f), euler_angles[0]*(float)M_PI/180.0f);
    Matrix3 rot_mat3 = Geometry::axisRotationMatrix(Vector3D(0.0f, 0.0f, 1.0f), euler_angles[2]*(float)M_PI/180.0f);

    std::vector<Vector3D> result;
    Vector3D xbasis = Vector3D(1.0f, 0.0f, 0.0);
    Vector3D ybasis = Vector3D(0.0f, 1.0f, 0.0);
    Vector3D zbasis = Vector3D(0.0f, 0.0f, 1.0f);

    xbasis = rot_mat2 * xbasis;
    xbasis = rot_mat1 * xbasis;
    xbasis = rot_mat3 * xbasis;

    ybasis = rot_mat2 * ybasis;
    ybasis = rot_mat1 * ybasis;
    ybasis = rot_mat3 * ybasis;

    zbasis = rot_mat2 * zbasis;
    zbasis = rot_mat1 * zbasis;
    zbasis = rot_mat3 * zbasis;

    xbasis.normalise();
    ybasis.normalise();
    zbasis.normalise();

    result.push_back(xbasis);
    result.push_back(ybasis);
    result.push_back(zbasis);
    return result;
}

std::vector<Vector3D> getGripperWorldOrientation(Vector3D approach_vector){
    std::vector<Vector3D> result;

    Vector3D dz = approach_vector;
    Vector3D tmp_dy(0.0f, 1.0f, 0.0f);

    Vector3D dx = tmp_dy.crossProduct(dz);
    Vector3D dy = dz.crossProduct(dx);

    dx.normalise();
    dy.normalise();
    dz.normalise();

    Matrix3 rot_mat = Geometry::axisRotationMatrix(dz, -5.0f*(float)M_PI/180.0f);
    dx = rot_mat * dx;
    dy = rot_mat * dy;

    result.push_back(dx);
    result.push_back(dy);
    result.push_back(dz);

    return result;
}

std::vector<Vector3D> positionAndEulerToPose(Vector3D position, std::vector<float> euler_angles){
    std::vector<Vector3D> result;
    std::vector<Vector3D> orientation = Control::gripperEulerAnglesToOrientation(euler_angles);

    result.push_back(position);
    result.push_back(orientation[0]);
    result.push_back(orientation[1]);
    result.push_back(orientation[2]);
    return result;
}

std::vector<float> eulerAnglesFromPose(std::vector<Vector3D> pose){
    std::vector<Vector3D> orientation;
    orientation.push_back(pose[1]);
    orientation.push_back(pose[2]);
    orientation.push_back(pose[3]);
    return gripperOrientationToEulerAngles(orientation);
}

void Control::moveArmToWorldPoint(Vector3D point, Vector3D approach){
    point = Common::worldPointToArmSpace(point);
    point.scale(0.1f);

    std::vector<float> cur_joints = Arm::getArm()->requestCurrentAngles();
    IKFitnessFunctionObjectPickup *pickup_function =
        new IKFitnessFunctionObjectPickup(point, cur_joints);

    ArmInverseKinematics ik;
    IKResult ik_result = ik.getRequiredJoints(pickup_function);

    Arm::getArm()->moveJointsAbs(ik_result.joints);

    delete pickup_function;
}

void Control::moveArmToPoint(std::vector<Vector3D> pose){
    std::vector<Vector3D> orientation;
    orientation.push_back(pose.at(1));
    orientation.push_back(pose.at(2));
    orientation.push_back(pose.at(3));

    std::vector<float> euler_angles = gripperOrientationToEulerAngles(orientation);

    Arm::getArm()->moveToPoint(pose[0].x, pose[0].y, pose[0].z,
    						   euler_angles[0], euler_angles[1], euler_angles[2]);

}

void Control::moveArmOutOfTheWay(void){
    std::cout << "Moving arm out of the way." << std::endl;
    //Arm::getArm()->releaseHand();
    std::vector<float> joints = Arm::getArm()->requestCurrentAngles();
    joints[0] = 90.0f;
    joints[1] = -30.0f;
    joints[4] = 0.0f;
    /*
    joints.push_back(70.0f);
    joints.push_back(-37.0f);
    joints.push_back(122.0f);
    joints.push_back(0.0f);
    joints.push_back(0.0f);
    joints.push_back(0.0f);
    safeMoveToJoints(joints);
    */

    Arm::getArm()->moveJointsAbs(joints);
    //Arm::getArm()->moveToPoint(76, 314, 332, 129, 51, 138);
}

void Control::safeMoveToJoints(std::vector<float> joints){
    if(joints[5] < -180.0f){
        joints[5] += 360.0f; 
    }
    if(joints[5] > 180.0f){
        joints[5] -= 360.0f;
    }

    std::vector<float> stage1_joints = Arm::getArm()->requestCurrentAngles();
    stage1_joints[0] = 0.31f;
    stage1_joints[1] = -30.0f;
    stage1_joints[5] = 0.0f;

    
	std::vector<float> stage2_joints = joints;
	stage2_joints[0] = 0.31f;
	stage2_joints[1] = -30.0f;
	stage2_joints[5] = 0.0f;

    //Arm::getArm()->moveJointsAbs(stage1_joints);
    Arm::getArm()->moveJointsAbs(stage2_joints);
	Arm::getArm()->moveJointsAbs(joints);
}

void Control::filterWorkspaceFeatures(std::vector<StereoFeature> &features){
    std::vector<StereoFeature> filtered_features;

    for(unsigned i = 0; i < features.size(); i++){
        if(!(features[i].position.x < -25.0f || features[i].position.x > 25.0f ||
             features[i].position.y < 20.0f || features[i].position.y > 70.0f ||
             features[i].position.z < 5.0f || features[i].position.z > 50.0f)){
            filtered_features.push_back(features[i]);
        }
    }

    features = filtered_features;
}

