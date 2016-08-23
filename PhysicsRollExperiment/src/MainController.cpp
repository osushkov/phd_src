
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
#include "Visualisation/WheeledBoxRenderObject.h"
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
#include "Physics/CompoundPhysicsObject.h"

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

static bool tryLoadExperiments(std::string filepath, Vector3D ground_plane_normal, std::vector<Experiment*> &experiments);
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
    /*
    SceneRenderer::instance().initialise(false);
    SceneRenderer::instance().clearObjects();

    PhysicsWorld *phys_world = new PhysicsWorld(0);
    phys_world->initialise();

    ObjectPhysModel main_obj(SuperQuadric(0.1f, 0.1f, 4.0f, 3.0f, 2.0f), 2.0, Vector3D(0.0f, 0.0f, 0.0f), 0.1f);
    ObjectPhysModel wheel0(SuperQuadric(1.0f, 1.0f, 1.0f, 1.0f, 1.0f), 1.0, Vector3D(0.0f, 0.0f, 0.0f), 0.1f);
    ObjectPhysModel wheel1(SuperQuadric(1.0f, 1.0f, 1.0f, 1.0f, 1.0f), 1.0, Vector3D(0.0f, 0.0f, 0.0f), 0.1f);
    ObjectPhysModel wheel2(SuperQuadric(1.0f, 1.0f, 1.0f, 1.0f, 1.0f), 1.0, Vector3D(0.0f, 0.0f, 0.0f), 0.1f);

    std::vector<Vector3D> rel_positions;
    rel_positions.push_back(Vector3D(0.0f, 0.0f, 0.0f));
    rel_positions.push_back(Vector3D(4.0f, 0.0f, 3.0f));
    rel_positions.push_back(Vector3D(-3.0f, 1.5f, -2.0f));
    rel_positions.push_back(Vector3D(-3.0f, 1.5f, -2.0f));

    std::vector<PhysicsObject*> phys_objects;
    phys_objects.push_back(main_obj.getPhysicsObject());
    phys_objects.push_back(wheel0.getPhysicsObject());
    //phys_objects.push_back(wheel1.getPhysicsObject());
    //phys_objects.push_back(wheel2.getPhysicsObject());

    std::vector<RenderObject*> render_objects;
    render_objects.push_back(main_obj.getRenderObject());
    render_objects.push_back(wheel0.getRenderObject());
    //render_objects.push_back(wheel1.getRenderObject());
    //render_objects.push_back(wheel2.getRenderObject());


    CompoundPhysicsObject *compound_object = new CompoundPhysicsObject(phys_objects, rel_positions);
    compound_object->addToWorld(phys_world);

    Transform new_pose;
    new_pose.quaternion = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
    new_pose.mat = new_pose.quaternion.toMatrix();
    new_pose.shift = Vector3D(0.0f, 0.0f, 20.0f);

    for(unsigned i = 0; i < phys_objects.size(); i++){
        Transform cur_pose = new_pose;
        Vector3D pos_adj = rel_positions[i];
        pos_adj.z = -pos_adj.z;
        cur_pose.shift = cur_pose.shift + pos_adj;
        phys_objects[i]->setPose(cur_pose);
        phys_objects[i]->activate();
    }

    for(unsigned i = 0; i < render_objects.size(); i++){
        SceneRenderer::instance().addObject(render_objects[i]);
    }

    SceneRenderer::instance().signal();
    for(unsigned i = 0; i < 500; i++){
        std::cout << i << std::endl;
        
        for(unsigned j = 0; j < render_objects.size(); j++){
            Transform cur_transform = phys_objects[j]->getTransform();
            cur_transform.quaternion.print();
            cur_transform.shift.print();
            render_objects[j]->setTransform(cur_transform);            
        }
        

        SceneRenderer::instance().signal();
        phys_world->step();
        getchar();
    }


    phys_world->clear();
    //delete phys_world;
    */
    
}

void MainController::experimentOnObject(std::string object_name){
    SceneRenderer::instance().initialise(true);

    float ramp_theta = 20.0f * M_PI/180.0f;
    Vector3D ramp_normal(0.0f, 0.0f, 1.0f);
    Matrix3 rot_mat = Geometry::axisRotationMatrix(Vector3D(0.0f, 1.0f, 0.0f), ramp_theta);
    ramp_normal = rot_mat * ramp_normal;

    std::vector<ObjectPhysModel> possible_phys_models = createPossiblePhysModels();
    std::vector<Experiment*> available_experiments = createAvailableExperiments(ramp_normal);
   
    ExperimentCoordinator::instance().initialise(possible_phys_models, available_experiments, ramp_theta);

    Object *obj = new Object(object_name);
    ExperimentCoordinator::instance().run(obj);

    //saveExperiments("data/experiments.dat", available_experiments);
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

    Vector3D box_params;
    std::vector<Vector3D> stub_params;
    std::vector<std::pair<float,float> > wheel_params;

    Transform box_transform;
    std::vector<Transform> stub_transforms;
    std::vector<Transform> wheel_transforms;

    box_params = Vector3D(5.0f, 3.0f, 1.5f);
    stub_params.push_back(Vector3D(1.0f, 1.0f, 1.0f));
    wheel_params.push_back(std::pair<float,float>(1.0f, 2.0f));

    box_transform = identityTransform();
   
    Transform stub_transform = identityTransform();
    stub_transform.shift = Vector3D(0.0f, 0.0f, -2.5f);
    stub_transforms.push_back(stub_transform);

    Transform wheel_transform = identityTransform();
    wheel_transform.shift = Vector3D(3.0f, 0.0f, -2.5f);
    wheel_transforms.push_back(wheel_transform);

    WheeledBoxRenderObject *wheeled_box = new WheeledBoxRenderObject(box_params, stub_params, wheel_params);
    wheeled_box->setBoxTransform(box_transform);
    wheeled_box->setStubTransforms(stub_transforms);
    wheeled_box->setWheelTransforms(wheel_transforms);

	SceneRenderer::instance().addObject(wheeled_box);
}

std::vector<ObjectPhysModel> MainController::createPossiblePhysModels(void){
    std::vector<ObjectPhysModel> result;

    std::vector<std::pair<float,float> > axle_frictions;
    std::vector<Vector3D> axle_orientations;
    std::vector<std::pair<float,float> > wheel_stub_xpositions;

    axle_frictions.push_back(std::pair<float,float>(0.15f, 0.15f));
    axle_frictions.push_back(std::pair<float,float>(0.15f, 1.0f));
    axle_frictions.push_back(std::pair<float,float>(1.0f, 0.15f));
    //axle_frictions.push_back(std::pair<float,float>(0.9f, 0.9f));


    axle_orientations.push_back(Vector3D(0.0f, 1.0f, 0.0f));
    axle_orientations.push_back(Vector3D(1.0f, 0.0f, 0.0f));
    //axle_orientations.push_back(Vector3D(-0.707f, 0.707f, 0.0f));

    wheel_stub_xpositions.push_back(std::pair<float,float>(-2.6f, 4.3f));
    //wheel_stub_xpositions.push_back(std::pair<float,float>(2.6f, -4.3f));

    // This is where for testing I create the actual model it is trying to find.
    std::vector<WheeledBoxPhysicsObject::WheelParams> wheel_params;
    std::vector<WheeledBoxPhysicsObject::StubParams> stub_params;

    WheeledBoxPhysicsObject::WheelParams wheel;
    wheel.width = 1.5f;
    wheel.radius = 1.5f;
    wheel.mass = 20.0f;
    wheel.friction = 0.6f;
    wheel.damping = 0.35f;
    wheel.axle = Vector3D(0.0f, 1.0f, 0.0f);

    wheel.position = Vector3D(-2.6f, 2.35f, -4.5f);
    wheel_params.push_back(wheel);

    wheel.position = Vector3D(-2.6f, -2.35f, -4.5f);
    wheel_params.push_back(wheel);


    WheeledBoxPhysicsObject::StubParams stub;
    stub.size = Vector3D(1.5f, 1.8f, 1.4f);
    stub.mass = 20.0f;
    stub.friction = 0.7f;

    stub.position = Vector3D(4.3f, 2.0f, -4.3f);
    stub_params.push_back(stub);

    stub.position = Vector3D(4.3f, -2.0f, -4.3f);
    stub_params.push_back(stub);

    for(unsigned i = 0; i < axle_frictions.size(); i++){
        for(unsigned j = 0; j < axle_orientations.size(); j++){
            for(unsigned k = 0; k < wheel_stub_xpositions.size(); k++){
                wheel_params[0].damping = axle_frictions[i].first;
                wheel_params[1].damping = axle_frictions[i].second;
                
                wheel_params[0].axle = axle_orientations[j];
                wheel_params[1].axle = axle_orientations[j];

                if(j == 0){
                    wheel_params[0].position = Vector3D(wheel_stub_xpositions[k].first, 2.35f, -4.5f);
                    wheel_params[1].position = Vector3D(wheel_stub_xpositions[k].first, -2.35f, -4.5f);
                }
                else{
                    wheel_params[0].position = Vector3D(-3.4, 1.0f, -4.5f);
                    wheel_params[1].position = Vector3D(-3.4, -3.0f, -4.5f);
                }

                stub_params[0].position = Vector3D(wheel_stub_xpositions[k].second, 2.0f, -4.3f);
                stub_params[1].position = Vector3D(wheel_stub_xpositions[k].second, -2.0f, -4.3f);

                result.push_back(ObjectPhysModel(Vector3D(5.9f, 4.15f, 2.8f), 150.0f, wheel_params, stub_params));
            }
        }
    }

    std::cout << "Number of models: " << result.size() << std::endl;
    return result;
}

bool tryLoadExperiments(std::string filepath, Vector3D ground_plane_normal, std::vector<Experiment*> &experiments){
    std::ifstream in_file(filepath.c_str());
    if(!in_file.good() || in_file.bad() || in_file.fail() || !in_file.is_open()){
        return false;
    }

    std::cout << "Trying to load experiments" << std::endl;

    unsigned num;
    in_file >> num;

    experiments.clear();
    for(unsigned i = 0; i < num; i++){
        Experiment *new_experiment = new DropExperiment(5.0f, 0.0f, ground_plane_normal);
        new_experiment->load(in_file);
        experiments.push_back(new_experiment);
    }

    return true;
}

bool saveExperiments(std::string filepath, std::vector<Experiment*> experiments){
    std::ofstream out_file(filepath.c_str());

    out_file << experiments.size() << std::endl;
    for(unsigned i = 0; i < experiments.size(); i++){
        experiments[i]->save(out_file);
    }

    return true;
}

std::vector<Experiment*> MainController::createAvailableExperiments(Vector3D ground_plane_normal){
    std::vector<Experiment*> result;
    
    if(!tryLoadExperiments("data/experiments.dat", ground_plane_normal, result)){
        std::cout << "Generting Experiments" << std::endl;
        const unsigned num_heights = 7;//6;
        const unsigned num_orientations = 30;
        const float min_height = 2.0f;
        const float max_height = 8.0f;

        for(unsigned i = 0; i < num_heights; i++){
            for(unsigned j = 0; j < num_orientations; j++){
                float height = min_height + (max_height - min_height)*i/(float)num_heights;
                float angle = ((float)j/(float)num_orientations * 2.0f * M_PI) - M_PI;

                result.push_back(new DropExperiment(height, angle, ground_plane_normal));
            }
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
