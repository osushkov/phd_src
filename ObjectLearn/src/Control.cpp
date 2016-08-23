
#include "Control.h"

#include <vector>
#include <iostream>
#include <list>
#include <pthread.h>

#include "ObjectFinder.h"
#include "Arm/Arm.h"
#include "Arm/ArmInverseKinematics.h"
#include "Arm/IKFitnessFunctionNearest.h"
#include "Arm/IKFitnessFunctionObjectPickup.h"
#include "Settings.h"
#include "Util/Common.h"
#include "Util/Geometry.h"
#include "ObjectLearner/ObjectLearner.h"
/*
static void* controlThread(void *thread_arg);

static void bringToStartPos(void);
static void bringToEndPos(void);

static void grabObject(DepthFeaturePublisher::ObjectOfInterest &target);
static void liftObject(void);
static void releaseObject(void);
static void moveArmOutOfTheWay(void);
*/

/*
static DepthFeaturePublisher::ObjectOfInterest chooseTargetGrabObject
    (const std::vector<DepthFeaturePublisher::ObjectOfInterest> &objects);



void Control::initialise(void){
    moveArmOutOfTheWay();
}


void Control::perform(void){
    pthread_create(&control_thread_id, NULL, &controlThread, (void *)NULL);
}


void bringToStartPos(void){
    Arm::getArm()->moveToPoint(200.0f, 310.0f, 130.0f, -170.0f, 47.0f, -144.0f);
}

void bringToEndPos(void){
    Arm::getArm()->moveToPoint(415.0f, 310.0f, 240.0f, -170.0f, 47.0f, -144.0f);
}
*/


std::vector<float> gripperOrientationToEulerAngles(std::vector<Vector3D> xyz_orientation){
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

std::vector<Vector3D> Control::gripperEulerAnglesToOrientation(std::vector<float> euler_angles){
    std::vector<Vector3D> rot_mat1 =
        Geometry::axisRotationMatrix(Vector3D(0.0f, 1.0f, 0.0f), euler_angles[1]*M_PI/180.0f);

    std::vector<Vector3D> rot_mat2 =
        Geometry::axisRotationMatrix(Vector3D(1.0f, 0.0f, 0.0f), euler_angles[0]*M_PI/180.0f);

    std::vector<Vector3D> rot_mat3 =
        Geometry::axisRotationMatrix(Vector3D(0.0f, 0.0f, 1.0f), euler_angles[2]*M_PI/180.0f);

    std::vector<Vector3D> result;
    Vector3D xbasis = Vector3D(1.0f, 0.0f, 0.0);
    Vector3D ybasis = Vector3D(0.0f, 1.0f, 0.0);
    Vector3D zbasis = Vector3D(0.0f, 0.0f, 1.0f);

    xbasis.matrixMultLeft(rot_mat2);
    xbasis.matrixMultLeft(rot_mat1);
    xbasis.matrixMultLeft(rot_mat3);

    ybasis.matrixMultLeft(rot_mat2);
    ybasis.matrixMultLeft(rot_mat1);
    ybasis.matrixMultLeft(rot_mat3);

    zbasis.matrixMultLeft(rot_mat2);
    zbasis.matrixMultLeft(rot_mat1);
    zbasis.matrixMultLeft(rot_mat3);

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

    std::vector<Vector3D> rot_mat = Geometry::axisRotationMatrix(dz, -5.0f*M_PI/180.0f);
    dx.matrixMultLeft(rot_mat);
    dy.matrixMultLeft(rot_mat);

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

    std::cout << "want to pick up: " << std::endl;
    point.print();

    std::vector<float> cur_joints = Arm::getArm()->requestCurrentAngles();
    IKFitnessFunctionObjectPickup *pickup_function =
        new IKFitnessFunctionObjectPickup(point, cur_joints);

    ArmInverseKinematics ik;
    IKResult ik_result = ik.getRequiredJoints(pickup_function);
    /*
    ArmForwardKinematics fk;
    std::vector<Vector3D> resultant_pose = fk.getArmPoint(ik_result.joints);
    std::vector<Vector3D> current_pose = fk.getArmPoint(Arm::getArm()->requestCurrentAngles());

    std::vector<float> resultant_euler = eulerAnglesFromPose(resultant_pose);
    std::vector<float> current_euler = eulerAnglesFromPose(current_pose);

    for(unsigned i = 1; i <= 2; i++){
        float a = (float)i/2.0f;

        std::vector<float> interim_euler;
        for(unsigned j = 0; j < resultant_euler.size(); j++){
            interim_euler.push_back(a*resultant_euler[i] + (1.0f-a)*current_euler[i]);
        }

        std::vector<Vector3D> interim_orientation = gripperEulerAnglesToOrientation(interim_euler);
        Vector3D interim_position = a*resultant_pose[0] + (1.0f-a)*current_pose[0];

        std::vector<Vector3D> interim_pose;
        interim_pose.push_back(interim_position);
        interim_pose.push_back(interim_orientation[0]);
        interim_pose.push_back(interim_orientation[1]);
        interim_pose.push_back(interim_orientation[2]);

        interim_pose[1].normalise();
        interim_pose[2].normalise();
        interim_pose[3].normalise();

        IKFitnessFunctionNearest *nearest_function =
            new IKFitnessFunctionNearest(interim_pose, cur_joints);

        IKResult interim_result = ik.getRequiredJoints(nearest_function);
        Arm::getArm()->moveJointsAbs(interim_result.joints);
        std::cout << i << std::endl;
        delete nearest_function;
    }
*/
    Arm::getArm()->moveJointsAbs(ik_result.joints);

    delete pickup_function;
/*
    sleep(10);
    exit(0);
    std::vector<Vector3D> orientation;
    orientation.push_back(ik_result.result_pose[1]);
    orientation.push_back(ik_result.result_pose[2]);
    orientation.push_back(ik_result.result_pose[3]);
    std::vector<float> euler_angles = gripperOrientationToEulerAngles(orientation);
    Arm::getArm()->moveToPoint(ik_result.result_pose[0].x,
                               ik_result.result_pose[0].y,
                               ik_result.result_pose[0].z,
                               euler_angles[0],
                               euler_angles[1],
                               euler_angles[2]);*/

    return;
/*
    Vector3D pos = Common::worldPointToArmSpace(point);
    pos.scale(0.1f);



    cur_pos.print();
    pos.print();
    Vector3D to_vec = pos - cur_pos;
    for(unsigned i = 0; i < 50; i++){
        Vector3D tp = cur_pos + (i/50.0f)*to_vec;
        std::cout << "ik" << std::endl;
        std::vector<float> joints = ik.getRequiredJoints(tp);
        std::cout << "moving" << std::endl;
        Arm::getArm()->moveJointsAbs(joints);
    }

    sleep(10);*/
    /*
    approach = Common::worldPointToArmSpace(point + approach) - pos;
    approach.normalise();


    std::vector<Vector3D> orientation = getGripperWorldOrientation(approach);

    std::vector<float> euler_angles = gripperOrientationToEulerAngles(orientation);
    Arm::getArm()->moveToPoint(pos.x, pos.y, pos.z,
                               euler_angles[0], euler_angles[1], euler_angles[2]);*/
}

void Control::moveArmToWorldPoint(std::vector<Vector3D> pose){
    /*
    std::vector<float> euler_angles;
    euler_angles.push_back(123.34);
    euler_angles.push_back(48.62);
    euler_angles.push_back(117.42);
    std::vector<Vector3D> orientation = gripperEulerAnglesToOrientation(euler_angles);
    for(unsigned i = 0; i < orientation.size(); i++){
        orientation[i].print();
    }*/

    assert(pose.size() == 4);

    Vector3D pos = pose.at(0);

    std::vector<Vector3D> orientation;
    orientation.push_back(pose.at(1));
    orientation.push_back(pose.at(2));
    orientation.push_back(pose.at(3));

    std::vector<float> euler_angles = gripperOrientationToEulerAngles(orientation);
    assert(euler_angles.size() == 3);

    while(!Arm::getArm()->amAtPoint(pos.x, pos.y, pos.z,
                       euler_angles.at(0), euler_angles.at(1), euler_angles.at(2), 50.0f)){
        Arm::getArm()->moveToPoint(pos.x, pos.y, pos.z,
                               euler_angles.at(0),
                               euler_angles.at(1),
                               euler_angles.at(2));

        usleep(100000);
    }

}

/*
void grabObject(DepthFeaturePublisher::ObjectOfInterest &target){
    std::vector<Vector3D> bounding_rectangle = target.bounding_box;

    Vector3D cm = Geometry::centreOfMass(bounding_rectangle);
    cm = cm + Vector3D(0.0f, -4.0f, 0.0f); // dodgy adjustment to deal with miscalibrated vision

    Vector3D main_axis = Vector3D(target.second_moment.x, target.second_moment.y, 0.0f);
    main_axis.print();
    //Geometry::findMainAxis(bounding_rectangle);
    float axis_length = 13.0f; //main_axis.length();
    main_axis.normalise();

    Vector3D approach_vector(1.0f, 0.5f, 0.0f); // ideal approach vector
    approach_vector.normalise();

    // Make sure that the main axis of the object is aligned with the idealised
    // approach vector to within 90deg, if not flip it (so as not to approach
    // from the opposite side). If the object is small, can approach from any
    // angle, just use the ideal approach vector.
    if(axis_length > 12.0f){
        if(main_axis.dotProduct(approach_vector) < 0.0f){
            main_axis.scale(-1.0f);
        }
        approach_vector = main_axis;
    }
    else{
        std::cout << "Object is too small, using ideal approach vector" << std::endl;
    }

    approach_vector.z -= 0.2f; // slightly tilt the hand so that the wrist doesnt hit the ground

    Vector3D preset_point = cm - 10.0f*approach_vector;
    approach_vector = Common::worldPointToArmSpace(cm + approach_vector) - Common::worldPointToArmSpace(cm);
    approach_vector.normalise();

    cm = Common::worldPointToArmSpace(cm);
    preset_point = Common::worldPointToArmSpace(preset_point);

    std::vector<Vector3D> world_space_gripper = getGripperWorldOrientation(approach_vector);
    std::vector<float> euler_angles = gripperOrientationToEulerAngles(world_space_gripper);

    float xrot = euler_angles[0];
    float yrot = euler_angles[1];
    float zrot = euler_angles[2];

    Arm::getArm()->openHand(1.0f);
    Arm::getArm()->moveToPoint(preset_point.x, 340.0f, preset_point.z, xrot, yrot, zrot);
    Arm::getArm()->moveToPoint(cm.x, 340.0f, cm.z, xrot, yrot, zrot);
    Arm::getArm()->closeHand(0.95f);
    sleep(1); // wait for the hand to close
}

void liftObject(void){
    std::vector<float> cur_pos =  Arm::getArm()->requestCurrentPos();
    assert(cur_pos.size() == 6);
    Arm::getArm()->moveToPoint(cur_pos[0], cur_pos[1]-50.0f, cur_pos[2],
                               cur_pos[3], cur_pos[4], cur_pos[5]);
}

void releaseObject(void){
    std::vector<float> cur_pos =  Arm::getArm()->requestCurrentPos();
    assert(cur_pos.size() == 6);
    Arm::getArm()->moveToPoint(cur_pos[0], 330.0f, cur_pos[2], cur_pos[3], cur_pos[4], cur_pos[5]);
    Arm::getArm()->openHand();
}

DepthFeaturePublisher::ObjectOfInterest chooseTargetGrabObject(const std::vector<DepthFeaturePublisher::ObjectOfInterest> &objects){
    assert(objects.size() > 0);
    return objects[0];
}

void* controlThread(void *thread_arg){
    GState::setGlobalState(SEARCHING_FOR_PICKUP_OBJECT);
    while(cur_frame < 10){ sleep(1); }

    std::vector<DepthFeaturePublisher::ObjectOfInterest> objects;// = DepthFeaturePublisher::getAllObjects();
    //while((objects = DepthFeaturePublisher::getAllObjects()).size() == 0){ sleep(1); }

    DepthFeaturePublisher::ObjectOfInterest target = chooseTargetGrabObject(objects);

    std::cout << "Grab object? :" << std::flush; getchar();
    GState::setGlobalState(PICKING_UP_OBJECT);
    cvDestroyWindow("DepthMap");
    grabObject(target);

    std::cout << "Lift object? :" << std::flush; getchar();
    liftObject();

    std::cout << "Move object to Start Pos? :" << std::flush; getchar();
    bringToStartPos();

    std::cout << "Move object to Pos2? :" << std::flush; getchar();
    GState::setGlobalState(LEARNING_OBJECT);
    bringToEndPos();
    object_learner->learn();
    GState::setGlobalState(NONE);

    std::cout << "Move object to Pos1? :" << std::flush; getchar();
    GState::setGlobalState(LEARNING_OBJECT);
    bringToStartPos();
    object_learner->learn();

    GState::setGlobalState(RELEASING_OBJECT);
    std::cout << "Release object? :" << std::flush; getchar();
    releaseObject();

    std::cout << "End Control? :" << std::flush; getchar();
    pthread_exit((void *)NULL);
}
*/

void Control::moveArmOutOfTheWay(void){
    /*
    std::vector<Vector3D> input;
    Vector3D a, b, c;

    Vector3D axis = Vector3D(1.0f, 0.0f, 1.0);
    axis.normalise();

    std::vector<Vector3D> rot_mat =
           Geometry::axisRotationMatrix(axis, 50.0*M_PI/180.0f);

    a = Vector3D(1.0f, 0.0f, 0.0f);
    b = Vector3D(0.0f, 1.0f, 0.0f);
    c = Vector3D(0.0f, 0.0f, 1.0f);

    a.matrixMultLeft(rot_mat);
    b.matrixMultLeft(rot_mat);
    c.matrixMultLeft(rot_mat);

    a.normalise();
    b.normalise();
    b.normalise();

    input.push_back(a);
    input.push_back(b);
    input.push_back(c);

    std::vector<float> euler_angles = gripperOrientationToEulerAngles(input);
    std::vector<Vector3D> basis_vectors = gripperEulerAnglesToOrientation(euler_angles);

    for(unsigned i = 0; i < euler_angles.size(); i++){
        std::cout << euler_angles[i] << " ";
    }
    std::cout << std::endl;

    for(unsigned i = 0; i < 3; i++){
        input[i].print();
        basis_vectors[i].print();
    }*/


    std::cout << "Moving arm out of the way." << std::endl;
    Arm::getArm()->openHand(0.8f);
    Arm::getArm()->moveToPoint(76, 314, 332, 129, 51, 138);
}

