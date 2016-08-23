
#include "MainController.h"
#include "Util/Common.h"
#include "Util/Geometry.h"
#include "Util/Timer.h"
#include "Visualisation/SceneRenderer.h"
#include "Visualisation/PointCloudRenderObject.h"
#include "Visualisation/SuperQuadricRenderObject.h"
#include "Visualisation/BoxRenderObject.h"
#include "Visualisation/MeshRenderObject.h"
#include "Visualisation/ArmRenderObject.h"
#include "Object/Object.h"
#include "Object/ObjectGrip.h"
#include "Object/ObjectExperimentTransfer.h"
#include "CameraController.h"
#include "Settings.h"
#include "Control.h"
#include "SuperQuadric.h"

#include "Physics/PhysicsWorld.h"
#include "Physics/PhysicsObject.h"
#include "Physics/BoxPhysicsObject.h"
#include "Physics/ConvexHullPhysicsObject.h"

#include "Experiment/DropExperiment.h"
#include "Experiment/ExperimentCoordinator.h"
#include "Experiment/ExperimentResultClassifier.h"

#include "Arm/Arm.h"
#include "Arm/ArmForwardKinematics.h"
#include "Arm/ArmInverseKinematics.h"
#include "Arm/IKFitnessFunctionNearest.h"


#define _USE_MATH_DEFINES
#include <math.h>
#include <cstdio>
#include <fstream>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/frames_io.hpp>

static bool tryLoadExperiments(std::string filepath, std::vector<Experiment*> &experiments);
static bool saveExperiments(std::string filepath, std::vector<Experiment*> experiments);

using namespace KDL;

MainController& MainController::instance(void){
    static MainController main_controller;
    return main_controller;
}

MainController::MainController() {

}


void MainController::initialise(void){

}

void MainController::reload(void){

    //initialise();
}

void MainController::examineObject(std::string object_name){
	Object object(object_name);
	object.examine();
}

void mouseHandleLeft(int event, int x, int y, int flags, void *param){
    switch(event){
    case CV_EVENT_LBUTTONDOWN:
        MainController::instance().mouseHandler(event, x, y, 0);
        break;

    }
}

void mouseHandleRight(int event, int x, int y, int flags, void *param){
    switch(event){
    case CV_EVENT_LBUTTONDOWN:
        MainController::instance().mouseHandler(event, x, y, 1);
        break;

    }
};

void MainController::sceneMatchObject(std::string object_name){
    std::cout << "MainController: sceneMatchObject " << object_name << std::endl;

#if 0
    SceneRenderer::instance().initialise();
/*
    ArmRenderObject *arm_ro = new ArmRenderObject();
    arm_ro->setJointAngles(Control::getArmRootPoses().at(1));
    SceneRenderer::instance().addObject(arm_ro);
    getchar();




    std::vector<Vector3D> arm_pose = afk.getArmPoint(Control::getArmRootPoses().at(2));
    std::vector<Vector3D> gripper_pose;
    gripper_pose.push_back(arm_pose[1]);
    gripper_pose.push_back(arm_pose[2]);
    gripper_pose.push_back(arm_pose[3]);
    std::vector<float> gripper_euler_angles = afk.gripperOrientationToEulerAngles(gripper_pose);

    Arm *arm = Arm::getArm();
    arm->moveToPoint(arm_pose[0].x, arm_pose[0].y, arm_pose[0].z,
    		gripper_euler_angles[0], gripper_euler_angles[1], gripper_euler_angles[2]);
    getchar();

    return;
*/
     Arm::getArm()->releaseHand();
   // Control::moveArmOutOfTheWay();

    Object *match_object = new Object(object_name);
    Transform obj_pose = Control::locateObject(match_object, 5);
    obj_pose.shift.print();
    getchar();
    ObjectGrip::GripData grip_data = ObjectGrip::gripObject(match_object, obj_pose, Quaternion(1.0f, 0.0f, 0.0f, 0.0), 15.0f);
    std::cout << "Displaying grip path" << std::endl;
    displayGripPath(grip_data, match_object, obj_pose);
    std::cout << "done" << std::endl;
    getchar();

    Control::safeMoveToJoints(grip_data.arm_joints);
    Arm::getArm()->openHand();
    Sleep(5000);
    Arm::getArm()->closeHand(0.8);
    Sleep(2000);
    Control::moveArmOutOfTheWay();
    Arm::getArm()->openHand();
    return;

    ArmForwardKinematics afk;
    std::vector<Vector3D> high_pose = afk.getArmPoint(grip_data.arm_joints);
    high_pose[0].y -= 40.0f;

    IKFitnessFunctionNearest *high_ff = new IKFitnessFunctionNearest(high_pose);
    ArmInverseKinematics ik;
    IKResult r = ik.performLocalSearch(high_ff, grip_data.arm_joints);
    std::cout << "high error: " << r.error << std::endl;
    Control::safeMoveToJoints(r.joints);
    std::cout << "MOVED!" << std::endl;
    /*
    //Arm::getArm()->moveJointsAbs(grip_data.root_joints);
    Control::safeMoveToJoints(grip_data.root_joints);
    sleep(2);


    std::vector<Vector3D> high_point = afk.getArmPoint(grip_data.arm_joints);
    high_point[0].y -= 40.0f;
    Control::moveArmToPoint(high_point);
    sleep(2);
    */

    std::cout << "Moving to: " << std::endl;
    Common::printVector(grip_data.arm_joints);
    Arm::getArm()->moveJointsAbs(grip_data.arm_joints);

    //Control::moveArmToJoints(grip_data.arm_joints);
    Arm::getArm()->softCloseHand();
#ifdef _WIN32
    Sleep(2000);
#else
    sleep(2);
#endif
    Arm::getArm()->moveJointsAbs(r.joints);
#ifdef _WIN32
    Sleep(2000);
#else
    sleep(2);
#endif
    Arm::getArm()->releaseHand();

    getchar();
    return;


    std::vector<std::pair<Quaternion,float> > experiments;
    for(unsigned i = 0; i < 5; i++){
        Matrix3 orientation_mat;
        orientation_mat.identity();

        float xrot = 2.0f*((float)rand()/(float)RAND_MAX-0.5f); // between -1 and 1
        float yrot = 2.0f*((float)rand()/(float)RAND_MAX-0.5f); // between -1 and 1
        Matrix3 xrot_mat = Geometry::axisRotationMatrix(Vector3D(1.0f, 0.0f, 0.0f), xrot*M_PI);
        Matrix3 yrot_mat = Geometry::axisRotationMatrix(Vector3D(0.0f, 1.0f, 0.0f), yrot*M_PI);

        orientation_mat = xrot_mat * orientation_mat;
        orientation_mat = yrot_mat * orientation_mat;

        Quaternion orientation_quaternion(orientation_mat);

        experiments.push_back(std::pair<Quaternion,float>(orientation_quaternion, 15.0f));
    }

    std::vector<float> best_exp_joints;
    float least_error = 0.0f;

    for(unsigned i = 0; i < experiments.size(); i++){
    	float error = 0.0f;
    	std::vector<float> cur_joints =
    			ObjectExperimentTransfer::transferObject(grip_data, match_object, obj_pose, experiments[i].first, experiments[i].second, error);

    	if(i == 0 || error < least_error){
    		least_error = error;
    		best_exp_joints = cur_joints;
    	}
    }
    displayGripPath(grip_data.arm_joints, best_exp_joints, match_object, ObjectGrip::armToObjectPoseTransform(obj_pose, grip_data));

    std::cout << "Done" << std::endl;
    getchar();
    return;

#endif

    //SceneRenderer::instance().initialise();

    Object *match_object = new Object(object_name);
    Transform obj_pose;

    bool pick_up_success = false;
    Transform arm_to_object = Control::pickUpObject(match_object, pick_up_success);
    getchar();
    
    //Control::moveToExperiment(match_object, arm_to_object, NULL);
    getchar();

    Arm::getArm()->releaseHand();
    return;

    PointCloudRenderObject *point_cloud = new PointCloudRenderObject(false, Vector3D(0.0f,1.0f,0.0f));
    SceneRenderer::instance().addObject(point_cloud);

    CameraController camera_controller(true, "", true, true);
    camera_controller.createViewWindows();

    while (camera_controller.isActive()) {
        if (!camera_controller.getNewFrame()) {
            break;
        }

        //std::cout << "!!??" << std::endl;
        std::vector<StereoFeature> raw_scene_features = camera_controller.getStereoFeatures();
        for(unsigned i = 0; i < raw_scene_features.size(); i++){
            raw_scene_features[i].position =
                Common::cameraPointToWorldSpace(raw_scene_features[i].position, 0.0f, -65.0f, 0.0f);
        }

        std::vector<StereoFeature> scene_features;
        for(unsigned i = 0; i < raw_scene_features.size(); i++){
            if(raw_scene_features[i].position.z > 10.0f && raw_scene_features[i].position.y < 50.0f){
                scene_features.push_back(raw_scene_features[i]);
            }
        }

        unsigned num_features_detected = 0;
        match_object->sceneMatch(scene_features, obj_pose, num_features_detected);

        std::vector<PointCloudPoint> point_cloud_points;
        for(unsigned i = 0; i < scene_features.size(); i++){
            PointCloudPoint new_point;
            new_point.pos = scene_features[i].position;
            point_cloud_points.push_back(new_point);
        }
        point_cloud->clearPoints();
        point_cloud->addPoints(point_cloud_points);


        match_object->render(obj_pose);
        

        IplImage *img_left = camera_controller.getLeftImage();
        IplImage *img_right = camera_controller.getRightImage();
        for(unsigned i = 0; i < scene_features.size(); i++){
            CvPoint lpos = cvPoint((int)scene_features[i].feature_left.x,
                                   (int)scene_features[i].feature_left.y);
            CvPoint rpos = cvPoint((int)scene_features[i].feature_right.x,
                                   (int)scene_features[i].feature_right.y);

            cvCircle(img_left, lpos, 3, CV_RGB(0, 0, 255), 1);
            cvCircle(img_right, rpos, 3, CV_RGB(0, 0, 255), 1);
        }
        
        if(camera_controller.showCameraViewAndContinue() == 'q'){
            break;
        }
    }

    delete match_object;
}

void MainController::physicsTest(void){ 
    
}

void MainController::experimentOnObject(std::string object_name){
    SceneRenderer::instance().initialise(false);

    std::vector<ObjectPhysModel> possible_phys_models = createPossiblePhysModels();
    std::vector<Experiment*> available_experiments = createAvailableExperiments();

    ExperimentCoordinator::instance().initialise(possible_phys_models, available_experiments);

    Object *obj = NULL; //new Object(object_name);
    ExperimentCoordinator::instance().run(obj);

    saveExperiments("data/experiments.dat", available_experiments);
}

void MainController::openHand(float amount, int speed){
	Arm *arm = Arm::getArm();
	assert(arm != NULL);
	arm->openHand(amount, speed);
}

void MainController::closeHand(float amount, int speed){
	Arm *arm = Arm::getArm();
	assert(arm != NULL);
	arm->closeHand(amount, speed);
}

void MainController::releaseHand(int speed){
	Arm *arm = Arm::getArm();
	assert(arm != NULL);
	arm->releaseHand(speed);
}

void MainController::test(void){
	SceneRenderer::instance().initialise();

    SuperQuadricRenderObject *sq_ro = new SuperQuadricRenderObject(SuperQuadric(0.1f, 1.0f, 2.0f, 2.0f, 2.0f));
	SceneRenderer::instance().addObject(sq_ro);
}

std::vector<ObjectPhysModel> MainController::createPossiblePhysModels(void){
    std::vector<ObjectPhysModel> result;
    /*
    SuperQuadric shape0(0.1f, 0.1f, 2.0f, 2.0f, 3.0f);
    SuperQuadric shape1(0.1f, 0.1f, 3.0f, 2.0f, 2.0f);
    SuperQuadric shape2(0.1f, 1.0f, 2.0f, 2.0f, 3.0f);
    SuperQuadric shape3(1.0f, 1.0f, 2.0f, 2.0f, 3.0f);
    SuperQuadric shape4(0.1f, 0.1f, 3.0f, 3.0f, 3.0f);
    SuperQuadric shape5(0.1f, 0.5f, 2.0f, 3.0f, 3.0f);

    result.push_back(ObjectPhysModel(shape0, 10.0f, Vector3D(0.0f, 0.0f, 0.0f), 0.1f));
    result.push_back(ObjectPhysModel(shape1, 10.0f, Vector3D(0.0f, 0.0f, 0.0f), 0.1f));
    result.push_back(ObjectPhysModel(shape2, 10.0f, Vector3D(0.0f, 0.0f, 0.0f), 0.1f));
    result.push_back(ObjectPhysModel(shape3, 10.0f, Vector3D(0.0f, 0.0f, 0.0f), 0.1f));
    result.push_back(ObjectPhysModel(shape4, 10.0f, Vector3D(0.0f, 0.0f, 0.0f), 0.1f));
    result.push_back(ObjectPhysModel(shape5, 10.0f, Vector3D(0.0f, 0.0f, 0.0f), 0.1f));
    */

    SuperQuadric shape0(0.1f, 0.1f, 2.75f, 4.2f, 5.95f);
    SuperQuadric shape1(0.1f, 1.0f, 2.5f, 2.5f, 7.0f);
/*
    result.push_back(ObjectPhysModel(shape1, 10.0f, Vector3D(0.0f, 0.0f, 4.5f), 0.1f));
    result.push_back(ObjectPhysModel(shape1, 10.0f, Vector3D(0.0f, 0.0f, 3.5f), 0.1f));
    result.push_back(ObjectPhysModel(shape1, 10.0f, Vector3D(0.0f, 0.0f, 2.5f), 0.1f));
    result.push_back(ObjectPhysModel(shape1, 10.0f, Vector3D(0.0f, 0.0f, 0.0f), 0.1f));
    result.push_back(ObjectPhysModel(shape1, 10.0f, Vector3D(0.0f, 0.0f, -2.5f), 0.1f));
    result.push_back(ObjectPhysModel(shape1, 10.0f, Vector3D(0.0f, 0.0f, -3.5f), 0.1f));
    result.push_back(ObjectPhysModel(shape1, 10.0f, Vector3D(0.0f, 0.0f, -4.5f), 0.1f));
*/

    result.push_back(ObjectPhysModel(shape1, 10.0f, Vector3D(0.0f, 0.0f, 0.0f), 0.1f));
    result.push_back(ObjectPhysModel(shape1, 10.0f, Vector3D(0.0f, 0.0f, 3.5f), 0.1f));
    result.push_back(ObjectPhysModel(shape1, 10.0f, Vector3D(0.0f, 0.0f, -3.5f), 0.1f));

    return result;
}

bool tryLoadExperiments(std::string filepath, std::vector<Experiment*> &experiments){
    std::ifstream in_file(filepath.c_str());
    if(!in_file.good() || in_file.bad() || in_file.fail() || !in_file.is_open()){
        return false;
    }

    ExperimentResultClassifier::instance().load(in_file);

    unsigned num;
    in_file >> num;

    experiments.clear();
    for(unsigned i = 0; i < num; i++){
        Experiment *new_experiment = new DropExperiment(0.0f, Quaternion(0.0f,0.0f,0.0f,0.0f), Vector3D(0.0f,0.0f,0.0f));
        new_experiment->load(in_file);
        experiments.push_back(new_experiment);
    }

    return true;
}

bool saveExperiments(std::string filepath, std::vector<Experiment*> experiments){
    std::ofstream out_file(filepath.c_str());

    ExperimentResultClassifier::instance().save(out_file);

    out_file << experiments.size() << std::endl;
    for(unsigned i = 0; i < experiments.size(); i++){
        experiments[i]->save(out_file);
    }

    return true;
}

std::vector<Experiment*> MainController::createAvailableExperiments(void){
#if 0
    std::vector<Experiment*> result;
    if(!tryLoadExperiments("data/experiments.dat", result)){
        std::vector<Vector3D> sphere_points = Geometry::pointsOnSphere(150);
        for(unsigned i = 0; i < sphere_points.size(); i++){
            Vector3D bvec0 = sphere_points[i];
            bvec0.normalise();

            Matrix3 orientation_mat = Geometry::getMatrixFromTo(bvec0, Vector3D(0.0f, 0.0f, 1.0f));
            Quaternion orientation_quaternion(orientation_mat);

            float height = 0.3f + rand()/(float)RAND_MAX * 1.0f;

            Experiment *new_experiment = new DropExperiment(height, orientation_quaternion, bvec0);
            result.push_back(new_experiment);
        }
    }
    else{
        std::cout << "Loaded experiments from file" << std::endl;
    }

    return result;
#endif

    std::vector<Experiment*> result;
    if(!tryLoadExperiments("data/experiments.dat", result)){
        for(unsigned i = 0; i < 100; i++){
            Vector3D bvec0((rand()/(float)RAND_MAX-0.5f)*2.0f,(rand()/(float)RAND_MAX-0.5f)*2.0f,(rand()/(float)RAND_MAX-0.5f)*2.0f);
            while(bvec0.length() > 1.0f){
                bvec0 = Vector3D((rand()/(float)RAND_MAX-0.5f)*2.0f,(rand()/(float)RAND_MAX-0.5f)*2.0f,(rand()/(float)RAND_MAX-0.5f)*2.0f);
            }
            bvec0.normalise();

            Matrix3 orientation_mat = Geometry::getMatrixFromTo(bvec0, Vector3D(0.0f, 0.0f, 1.0f));
            Quaternion orientation_quaternion(orientation_mat);

            float height = 0.3f * rand()/(float)RAND_MAX + 0.3f;

            Experiment *new_experiment = new DropExperiment(height, orientation_quaternion, bvec0);
            result.push_back(new_experiment);
        }
    }
    else{
        std::cout << "Loaded experiments from file" << std::endl;
    }

    return result;

}

void MainController::mouseHandler(int event, int x, int y, int left_or_right){
    /*CvPoint lpos = cvPoint(x, y);
    cvCircle(img_right, lpos, 3.0, CV_RGB(255, 0, 0), 1.0);
    cvShowImage("RightEye", img_right);
    cvWaitKey(5);
*/
}
