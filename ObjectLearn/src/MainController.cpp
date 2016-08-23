
#include "MainController.h"

#include "CorrelatedFrames/CorrelatedFrameLearner.h"
#include "Camera/Camera.h"
#include "Camera/RemoteCamera.h"
#include "Camera/PlaybackCamera.h"
#include "Camera/Recorder.h"
#include "Features/SIFT/sift.h"
#include "Features/SIFT/imgfeatures.h"
#include "Util/Common.h"
#include "Util/ParallelServer.h"
#include "Util/Timer.h"
#include "Util/PerfStats.h"
#include "Util/Semaphore.h"
#include "Util/Geometry.h"
#include "Util/ConvexHull.h"
#include "Util/MinCut.h"
#include "Features/StereoFeatureCorrelation.h"
#include "ObjectLearner/TraceTracker.h"
#include "ObjectLearner/FrameHistoryBuffer.h"
#include "Features/SIFTGenerator.h"
#include "ObjectFinder.h"
#include "Arm/Arm.h"
#include "Settings.h"
#include "Control.h"
#include "ObjectLearner/ObjectLearner.h"
#include "Console.h"
#include "Demos/BallPickup.h"
#include "Arm/Arm.h"
#include "Arm/ArmForwardKinematics.h"
#include "Visualisation/SceneRenderer.h"
#include "Visualisation/CylinderRenderObject.h"
#include "Visualisation/PointCloudRenderObject.h"
#include "Visualisation/LinkedPointsRenderObject.h"
#include "Visualisation/SuperQuadricRenderObject.h"
#include "Features/FeatureMemory/FeatureMemory.h"
#include "Reconstruction/ReconstructionManager.h"
#include "Reconstruction/SuperQuadric.h"
#include "Object.h"
#include "Evaluation/MemoryEvaluator.h"
#include "Features/FeatureMemory/ObjectSnapshotDBMono.h"
#include "Features/FeatureMemory/MemoryValidator.h"
#include "CameraController.h"
#include "SilhouetteMatch/ObjectShapeVerifier.h"
#include "SilhouetteMatch/SilhouetteEval.h"
#include "OptimiserClass/Common.h"

MainController& MainController::instance(void){
    static MainController main_controller;
    return main_controller;
}

MainController::MainController() : cur_frame(0) {

}


void MainController::initialise(void){
    cur_frame = 0;
}

void MainController::reload(void){

    //initialise();
}

void MainController::learnObject(std::string object_name){
    Object object(object_name);
    object.buildStage(OBJECT_BS_FULL);
}

void MainController::examineObject(std::string object_name){
	Object object(object_name);
	object.examine();
}

void MainController::correlateObjects(std::string object_name){
    std::cout << "MainController: correlateObjects - " << object_name << std::endl;
    std::string path = Settings::instance().getStringValue("general", "correlated_features_path");

    CorrelatedFrameLearner correlated_learner;
    correlated_learner.loadCorrelatedFrames(path);
    correlated_learner.processCorrelatedFrames(object_name);
}

void MainController::generateSIFT(std::string path){
    std::cout << "MainController: generateSIFT - " << path << std::endl;

    loadCamera(false, path);
    loadSIFTGenerator(true, path);

    createViewWindows();

    running = true;
    while (cam->isActive() && running) {
        if(!getNewFrame(true)){
            break;
        }

        for(unsigned i = 0; i < stereo_features.size(); i++){
            draw_features(img_left, &(stereo_features[i].feature_left), 1);
            draw_features(img_right, &(stereo_features[i].feature_right), 1);
        }

        showCameraViewAndContinue();
    }

    closeWindow("LeftEye");
    closeWindow("RightEye");

    delete sift_generator;
    delete cam;
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

static bool isCategoryPixel(std::vector<StereoFeature> &all_features, std::set<unsigned> &cat_features, Vector2D pixel_pos){
    std::set<unsigned>::iterator it;
    for(it = cat_features.begin(); it != cat_features.end(); ++it){
        Vector2D fpos(all_features[*it].feature_left.x, all_features[*it].feature_left.y);
        if((pixel_pos - fpos).length() <= sqrtf(2.0f*49.0f)){
            return true;
        }
    }

    return false;
}

static MinCut::Graph buildGraph(std::vector<StereoFeature> all_features, std::set<unsigned> arm_features){
    MinCut::resetNodeLabels();
    MinCut::Graph result;

    MinCut::GraphNode *arm_node = new MinCut::GraphNode;
    MinCut::GraphNode *background_node = new MinCut::GraphNode;

    arm_node->label = -2;
    background_node->label = -1;

    std::vector<MinCut::GraphNode*> pixel_nodes;
    std::set<unsigned> background_features;
    for(unsigned i = 0; i < all_features.size(); i++){
        if(all_features[i].position.length() > 65.0f){
            background_features.insert(i);
        }
    }

    std::set<unsigned> arm_pixel_nodes, background_pixel_nodes;
    for(int y = 0; y < 384/8; y++){
        for(int x = 0; x < 512/8; x++){
            MinCut::GraphNode* new_node = new MinCut::GraphNode;
            new_node->label = MinCut::getNextNodeLabel();

            if(isCategoryPixel(all_features, arm_features, Vector2D(x*8.0f,y*8.0f))){
                MinCut::GraphEdge *new_edge0 = new MinCut::GraphEdge;
                MinCut::GraphEdge *new_edge1 = new MinCut::GraphEdge;

                new_edge0->src = new_edge1->dst = new_node;
                new_edge0->dst = new_edge1->src = arm_node;
                new_edge0->weight = new_edge1->weight = 1000.0f;

                new_node->edges[arm_node->label] = new_edge0;
                arm_node->edges[new_node->label] = new_edge1;

                arm_pixel_nodes.insert(x + y*512/8);
            }
            else if(isCategoryPixel(all_features, background_features, Vector2D(x*8.0f,y*8.0f))){
                MinCut::GraphEdge *new_edge0 = new MinCut::GraphEdge;
                MinCut::GraphEdge *new_edge1 = new MinCut::GraphEdge;

                new_edge0->src = new_edge1->dst = new_node;
                new_edge0->dst = new_edge1->src = background_node;
                new_edge0->weight = new_edge1->weight = 1000.0f;

                new_node->edges[background_node->label] = new_edge0;
                background_node->edges[new_node->label] = new_edge1;

                background_pixel_nodes.insert(x + y*512/8);
            }

            pixel_nodes.push_back(new_node);
        }
    }

    std::cout << arm_pixel_nodes.size() << " " << background_pixel_nodes.size() << std::endl;
    for(int y = 0; y < 384/8; y++){
        for(int x = 0; x < 512/8; x++){
            MinCut::GraphNode* cur_node = pixel_nodes[x + y*512/8];

            for(int yoffset = -1; yoffset <= 1; yoffset++){
                for(int xoffset = -1; xoffset <= 1; xoffset++){
                    if(xoffset == 0 && yoffset == 0){ continue; }

                    int lx = x + xoffset;
                    int ly = y + yoffset;
                    if(lx < 0 || lx >= 512/8 || ly < 0 || ly >= 384/8){ continue; }
                    float dist = sqrtf(xoffset*xoffset + yoffset*yoffset);

                    MinCut::GraphNode* lnode = pixel_nodes[lx + ly*512/8];

                    MinCut::GraphEdge *new_edge0 = new MinCut::GraphEdge;
                    MinCut::GraphEdge *new_edge1 = new MinCut::GraphEdge;

                    new_edge0->src = new_edge1->dst = cur_node;
                    new_edge0->dst = new_edge1->src = lnode;
                    new_edge0->weight = new_edge1->weight = 1.0/dist;

                    if(arm_pixel_nodes.find(lx + ly*512/8) != arm_pixel_nodes.end() ||
                       background_pixel_nodes.find(lx + ly*512/8) != background_pixel_nodes.end()){
                        new_edge0->weight *= 10.0f;
                        new_edge1->weight *= 10.0f;
                    }

                    cur_node->edges[lnode->label] = new_edge0;
                    lnode->edges[cur_node->label] = new_edge1;
                }
            }
        }
    }

    result.nodes[-2] = arm_node;
    result.nodes[-1] = background_node;
    for(unsigned i = 0; i < pixel_nodes.size(); i++){
        assert(pixel_nodes[i]->label == (int)i);
        result.nodes[i] = pixel_nodes[i];
    }

    return result;
}

static bool isNearSet(Vector2D pos, std::set<int> label_set){
    std::set<int>::iterator it;
    for(it = label_set.begin(); it != label_set.end(); ++it){
        int x = *it%(512/8);
        int y = *it/(512/8);
        pos.x /= 8.0f;
        pos.y /= 8.0f;

        if((pos - Vector2D(x,y)).length() <= 1.0f){
            return true;
        }
    }

    return false;
}

void MainController::test(void){
    SuperQuadric base(0.1, 1.0f, 3.25, 3.25f, 6.4f, 1.0f, 1.0f);
    SuperQuadric sq(0.25f, 0.9f, 3.1, 3.28, 5.7, 0.93, 0.93);
    float avrg, sd, max;
    SuperQuadric::shapeDifference2(base, sq, avrg, sd, max);
    std::cout << "diff: " << avrg << " " << sd << " " << max << std::endl;
    return;

    std::vector<std::pair<std::string,std::string> > eval_images;
    eval_images.push_back(std::pair<std::string,std::string>("data/fitting_data/vcan_left0_base.png","data/fitting_data/vcan_right0_base.png"));
    eval_images.push_back(std::pair<std::string,std::string>("data/fitting_data/vcan_left1_base.png","data/fitting_data/vcan_right1_base.png"));
    eval_images.push_back(std::pair<std::string,std::string>("data/fitting_data/vcan_left2_base.png","data/fitting_data/vcan_right2_base.png"));
    eval_images.push_back(std::pair<std::string,std::string>("data/fitting_data/vcan_left3_base.png","data/fitting_data/vcan_right3_base.png"));
    //eval_images.push_back(std::pair<std::string,std::string>("data/fitting_data/vcan_left4_base.png","data/fitting_data/vcan_right4_base.png"));

    std::vector<std::pair<std::string,std::string> > match_images;
    match_images.push_back(std::pair<std::string,std::string>("data/fitting_data/vcan_left0.png","data/fitting_data/vcan_right0.png"));
    match_images.push_back(std::pair<std::string,std::string>("data/fitting_data/vcan_left1.png","data/fitting_data/vcan_right1.png"));
    match_images.push_back(std::pair<std::string,std::string>("data/fitting_data/vcan_left2.png","data/fitting_data/vcan_right2.png"));
    match_images.push_back(std::pair<std::string,std::string>("data/fitting_data/vcan_left3.png","data/fitting_data/vcan_right3.png"));
    //match_images.push_back(std::pair<std::string,std::string>("data/fitting_data/vcan_left4.png","data/fitting_data/vcan_right4.png"));


    SilhouetteEval silhouette_eval(eval_images, match_images);

    Object object("vcan_white");
    float error = silhouette_eval.evalObject(&object);
    std::cout << "silhouette error: " << error << std::endl;

    return;


    CameraController camera_controller(false, "data/vcan_rotate/", true, false);
    camera_controller.createViewWindows();

    ObjectSnapshotDB *arm_snapshot_db = ObjectSnapshotDB::getArmSnapshotDB();

    bool paused = false;
    while (camera_controller.isActive()) {
        if (!camera_controller.getNewFrame()) {
            break;
        }

        IplImage *img_left = camera_controller.getLeftImage();
        IplImage *img_right = camera_controller.getRightImage();

        std::vector<StereoFeature> stereo_features = camera_controller.getStereoFeatures();
        SceneMatchResult match_result = arm_snapshot_db->matchScene(stereo_features, false);
        MinCut::Graph graph = buildGraph(stereo_features, match_result.matched_scene_features);

        std::set<int> cut_labels;
        float min_cut_cost = MinCut::performMinCut(graph, cut_labels);
        std::cout << "min cut: " << min_cut_cost << " " << cut_labels.size() << std::endl;

        std::set<int>::iterator it;
        for(it = cut_labels.begin(); it != cut_labels.end(); ++it){
            int x = *it%(512/8);
            int y = *it/(512/8);
            x *= 8;
            y *= 8;

            CvPoint pos = cvPoint(x, y);
            cvCircle(img_left, pos, 1.0, CV_RGB(255, 0, 0), 1.0);
        }
        /*
        for(unsigned i = 0; i < stereo_features.size(); i++){
            CvScalar left_color = CV_RGB(0, 255, 0);
            CvScalar right_color = CV_RGB(0, 255, 0);
            if(match_result.matched_scene_features.find(i) == match_result.matched_scene_features.end()){
                right_color = CV_RGB(255, 0, 0);
            }
            if(isNearSet(Vector2D(stereo_features[i].feature_left.x,stereo_features[i].feature_left.y), cut_labels)){
                left_color = CV_RGB(255, 0, 0);
            }

            CvPoint pos = cvPoint(stereo_features[i].feature_left.x, stereo_features[i].feature_left.y);
            cvCircle(img_left, pos, 2.0, left_color, 1.0);

            pos = cvPoint(stereo_features[i].feature_right.x, stereo_features[i].feature_right.y);
            cvCircle(img_right, pos, 2.0, right_color, 1.0);
        }
        */


        char key = camera_controller.showCameraViewAndContinue();
        if(key == 'q'){
            break;
        }
        else if(key == 'p'){
            paused = !paused;
            camera_controller.setPaused(paused);
        }
    }
}

void MainController::generateMovement(std::string record_path){

    /*std::vector<float> euler_angles;
    euler_angles.push_back(136.0f);
    euler_angles.push_back(45.0f);
    euler_angles.push_back(130.0f);
    std::vector<Vector3D> gripper_orientation = Control::gripperEulerAnglesToOrientation(euler_angles);

    for(unsigned i = 0; i < gripper_orientation.size(); i++){
        gripper_orientation[i].print();
    }
    return;*/

    Arm::getArm()->closeHand(0.3f);
    sleep(5);
    Arm::getArm()->closeHand(0.85f);
    loadCamera(true, "");

    std::vector<Vector3D> positions;
    //positions.push_back(Vector3D(286.0f, 314.0f, 243.0f));
    positions.push_back(Vector3D(299.0f, 314.0f, 415.0f));
    positions.push_back(Vector3D(360.0f, 314.0f, 165.0f));
    positions.push_back(Vector3D(472.0f, 314.0f, 285.0f));
    //positions.push_back(Vector3D(354.0f, 314.0f, 343.0f));
    positions.push_back(Vector3D(213.0f, 314.0f, 243.0f));
    //positions.push_back(Vector3D(299.0f, 314.0f, 415.0f));


    std::vector<Vector3D> orientation;
    orientation.push_back(Vector3D(0.0f, 1.0f, 0.0f));
    orientation.push_back(Vector3D(0.0f, 0.0f, 1.0f));
    orientation.push_back(Vector3D(1.0f, 0.0f, 0.0f));
    std::vector<Vector3D> fix_mat = Geometry::axisRotationMatrix(orientation[2], -60.0f*M_PI/180.0f);
    for(unsigned i = 0; i < orientation.size(); i++){
        orientation[i].matrixMultLeft(fix_mat);
    }

    std::vector<Vector3D> pose;
    pose.push_back(positions[0]);
    pose.push_back(orientation[0]);
    pose.push_back(orientation[1]);
    pose.push_back(orientation[2]);

    float rot_offset = 22.5f;

    std::cout << "start?" << std::endl;
    getchar();

    Recorder *recorder = NULL;
    if(record_path.size() > 0){
        recorder = new Recorder(record_path, cam);
        recorder->startRecording();
    }

    for(unsigned i = 0; i < positions.size()+1; i++){
        pose[0] = positions[i%positions.size()];
        Control::moveArmToWorldPoint(pose);
/*
        fix_mat = Geometry::axisRotationMatrix(pose[3], rot_offset*M_PI/180.0f);
        for(unsigned j = 1; j < pose.size(); j++){
            pose[j].matrixMultLeft(fix_mat);
        }*/
    }

    if(recorder != NULL){
        recorder->stopRecording();
        delete recorder;
    }

    delete cam;
    Arm::getArm()->moveJointAbs(6, 0.0f);
}

void MainController::rotate(std::string record_path){
    Arm::getArm()->closeHand(0.0f);
    sleep(5);
    Arm::getArm()->closeHand(0.9f);
    getchar();

    // TODO: move the hand into a position suitable for observation.

    Arm::getArm()->moveJointAbs(6, -180.0f);
    while(fabs(Arm::getArm()->requestCurrentAngles().at(5) + 180.0f) > 5.0f){
        usleep(100000);
    }
    loadCamera(true, "");

    Recorder *recorder = NULL;

    if(record_path.size() > 0){
        recorder = new Recorder(record_path, cam, true);
        recorder->startRecording();
    }
    Arm::getArm()->moveJointAbs(6, 180.0f);
    while(fabs(Arm::getArm()->requestCurrentAngles().at(5) - 180.0f) > 5.0f){
        std::cout << Arm::getArm()->requestCurrentAngles().at(5) << std::endl;
        usleep(100000);
    }

    getchar();

    if(recorder != NULL){
        recorder->stopRecording();
        delete recorder;
    }

    delete cam;
}

void MainController::sceneMatchObject(std::string object_name){
    std::cout << "MainController: sceneMatchObject " << object_name << std::endl;

    SceneRenderer::instance().initialise();

    PointCloudRenderObject *point_cloud = new PointCloudRenderObject(false, Vector3D(0.0f,1.0f,0.0f));
    SceneRenderer::instance().addObject(point_cloud);

    std::vector<Object*> match_objects;
    match_objects.push_back(new Object(object_name));

    //CameraController camera_controller(true, "", true, true);
    CameraController camera_controller(false, "data/vcan_rotate/", true, false);
    camera_controller.createViewWindows();

    while (camera_controller.isActive()) {
        if (!camera_controller.getNewFrame()) {
            break;
        }


        std::vector<StereoFeature> scene_features = camera_controller.getStereoFeatures();
        std::vector<StereoFeature> cam_scene_features = scene_features;
        for(unsigned i = 0; i < scene_features.size(); i++){
            scene_features[i].position =
                Common::cameraPointToWorldSpace(scene_features[i].position, 0.0f, -65.0f, 0.0f);
        }

        std::vector<PointCloudPoint> point_cloud_points;
        for(unsigned i = 0; i < scene_features.size(); i++){
            PointCloudPoint new_point;
            new_point.pos = scene_features[i].position;
            point_cloud_points.push_back(new_point);
        }
        point_cloud->clearPoints();
        point_cloud->addPoints(point_cloud_points);


        for(unsigned i = 0; i < match_objects.size(); i++){
            Transform object_transform;
            if(match_objects[i]->sceneMatch(scene_features, object_transform)){
                match_objects[i]->render(object_transform);
            }
        }


        IplImage *img_left = camera_controller.getLeftImage();
        IplImage *img_right = camera_controller.getRightImage();
        for(unsigned i = 0; i < scene_features.size(); i++){
            CvPoint lpos = cvPoint(scene_features[i].feature_left.x,
                                   scene_features[i].feature_left.y);
            CvPoint rpos = cvPoint(scene_features[i].feature_right.x,
                                   scene_features[i].feature_right.y);

            cvCircle(img_left, lpos, 3.0, CV_RGB(0, 0, 255), 1.0);
            cvCircle(img_right, rpos, 3.0, CV_RGB(0, 0, 255), 1.0);
        }

        if(camera_controller.showCameraViewAndContinue() == 'q'){
            break;
        }
    }

    for(unsigned i = 0; i < match_objects.size(); i++){
        delete match_objects[i];
    }
}

void MainController::stitchTogether(std::string object0_name, std::string object1_name){
    Object *object0 = new Object(object0_name);
    Object *object1 = new Object(object1_name);

    std::vector<ModelFrame> object0_frames = object0->getModelFrames();
    std::vector<ModelFrame> object1_frames = object1->getModelFrames();

    std::vector<StereoFeature> object0_features;
    std::vector<StereoFeature> object1_features;

    for(unsigned i = 0; i < object0_frames.size(); i++){
        for(unsigned j = 0; j < object0_frames[i].features.size(); j++){
            object0_features.push_back(object0_frames[i].features[j]);
        }
    }
    for(unsigned i = 0; i < object1_frames.size(); i++){
        for(unsigned j = 0; j < object1_frames[i].features.size(); j++){
            object1_features.push_back(object1_frames[i].features[j]);
        }
    }

    ObjectSnapshot *object0_snapshot = new ObjectSnapshot("obj0", 0);
    object0_snapshot->addFeatures(object0_features);

    std::vector<SnapshotMatchResult> match_result;
    Transform approx_transform;
    float r = object0_snapshot->matchScene(object1_features, match_result, approx_transform, true);
    std::cout << "R:" << r << " " << object0_features.size() << " " << object1_features.size() << std::endl;

    SceneRenderer::instance().initialise();
    PointCloudRenderObject *point_cloud = new PointCloudRenderObject(false, Vector3D(0.0f,1.0f,0.0f));
    SceneRenderer::instance().addObject(point_cloud);

    std::vector<PointCloudPoint> point_cloud_points;
    for(unsigned i = 0; i < object0_features.size(); i++){
        PointCloudPoint new_point;
        new_point.pos = object0_features[i].position - approx_transform.secondary_shift;
        new_point.pos = approx_transform.mat*new_point.pos;
        new_point.pos = new_point.pos + approx_transform.shift;
        point_cloud_points.push_back(new_point);
    }
    for(unsigned i = 0; i < object1_features.size(); i++){
        PointCloudPoint new_point;
        new_point.pos = object1_features[i].position;
        point_cloud_points.push_back(new_point);
    }
    point_cloud->clearPoints();
    point_cloud->addPoints(point_cloud_points);

    for(unsigned i = 0; i < object0_frames.size(); i++){
        for(unsigned j = 0; j < object0_frames[i].features.size(); j++){
            object0_frames[i].features[j].position = object0_frames[i].features[j].position - approx_transform.secondary_shift;
            object0_frames[i].features[j].position = approx_transform.mat*object0_frames[i].features[j].position;
            object0_frames[i].features[j].position = object0_frames[i].features[j].position + approx_transform.shift;
        }
        object1_frames.push_back(object0_frames[i]);
    }

    std::ofstream object1_frames_file("blah.dat", std::ios::binary);
    unsigned num_frames = object1_frames.size();
    object1_frames_file.write((char*)&num_frames, sizeof(unsigned));

    for(unsigned i = 0; i < object1_frames.size(); i++){
        object1_frames_file.write((char*)&(object1_frames[i].view_direction), sizeof(Vector3D));

        unsigned num_features = object1_frames[i].features.size();
        object1_frames_file.write((char*)&num_features, sizeof(unsigned));
        for(unsigned j = 0; j < object1_frames[i].features.size(); j++){
            Common::writeFeature(object1_frames_file, object1_frames[i].features[j]);
        }
    }

    delete object0_snapshot;
    //delete object1_snapshot;

    delete object0;
    delete object1;

    getchar();
}

void MainController::silhouetteObject(std::string object_name){
    std::cout << "MainController: silhouetteObject " << object_name << std::endl;

    SceneRenderer::instance().initialise();

    PointCloudRenderObject *point_cloud = new PointCloudRenderObject(false, Vector3D(0.0f,1.0f,0.0f));
    SceneRenderer::instance().addObject(point_cloud);

    Object* match_object = new Object(object_name);

    //CameraController camera_controller(true, "", true, true);
    CameraController camera_controller(false, "data/vcan_surround/", true, false);
    camera_controller.createViewWindows();

    std::vector<std::pair<IplImage*,IplImage*> > img_pairs;
    std::vector<std::vector<StereoFeature> > img_features;

    unsigned frame_number = 0;
    while (camera_controller.isActive()) {
        if (!camera_controller.getNewFrame()) {
            break;
        }

        IplImage *img_left = camera_controller.getLeftImage();
        IplImage *img_right = camera_controller.getRightImage();
        std::vector<StereoFeature> scene_features = camera_controller.getStereoFeatures();

        bool frame_snapshot = false;
        if(frame_number%40 == 0){
            IplImage *left_copy = cvCreateImage(cvGetSize(img_left), IPL_DEPTH_8U, 3);
            IplImage *right_copy = cvCreateImage(cvGetSize(img_right), IPL_DEPTH_8U, 3);

            cvCopy(img_left, left_copy);
            cvCopy(img_right, right_copy);

            img_pairs.push_back(std::pair<IplImage*,IplImage*>(left_copy, right_copy));
            img_features.push_back(scene_features);
            frame_snapshot = true;
        }

#if 0
        std::vector<StereoFeature> cam_scene_features = scene_features;
        for(unsigned i = 0; i < scene_features.size(); i++){
            scene_features[i].position =
                Common::cameraPointToWorldSpace(scene_features[i].position, 0.0f, -65.0f, 0.0f);
        }

        std::vector<PointCloudPoint> point_cloud_points;
        for(unsigned i = 0; i < cam_scene_features.size(); i++){
            PointCloudPoint new_point;
            new_point.pos = cam_scene_features[i].position;
            point_cloud_points.push_back(new_point);
        }
        point_cloud->clearPoints();
        point_cloud->addPoints(point_cloud_points);


        Transform object_transform;
        if(match_object->sceneMatch(cam_scene_features, object_transform)){
            object_transform.shift.print();
            match_object->render(object_transform);
        }

        ObjectShapeVerifier shape_verifier;
        Mesh object_mesh = match_object->getObjectMesh();

        std::vector<std::pair<IplImage*,IplImage*> > images;
        images.push_back(std::pair<IplImage*,IplImage*>(img_left, img_right));
        shape_verifier.calculateModelSilhouetteScore(images, object_mesh, object_transform);


        IplImage* grey_img_left = cvCreateImage(cvGetSize(img_left), IPL_DEPTH_8U, 1);
        cvCvtColor(img_left, grey_img_left, CV_BGR2GRAY);

        IplImage* grey_img_right = cvCreateImage(cvGetSize(img_left), IPL_DEPTH_8U, 1);
        cvCvtColor(img_right, grey_img_right, CV_BGR2GRAY);
/*
        IplImage* left_edge_image = cvCreateImage(cvGetSize(img_left), IPL_DEPTH_8U, 1);
        IplImage* right_edge_image = cvCreateImage(cvGetSize(img_right), IPL_DEPTH_8U, 1);
        cvcanny(grey_img_left, left_edge_image, 50, 100);
        cvcanny(grey_img_right, right_edge_image, 50, 100);
*/

        for(unsigned i = 0; i < scene_features.size(); i++){
            CvPoint lpos = cvPoint(scene_features[i].feature_left.x,
                                   scene_features[i].feature_left.y);
            CvPoint rpos = cvPoint(scene_features[i].feature_right.x,
                                   scene_features[i].feature_right.y);

            cvCircle(img_left, lpos, 3.0, CV_RGB(0, 0, 255), 1.0);
            cvCircle(img_right, rpos, 3.0, CV_RGB(0, 0, 255), 1.0);
        }
#endif

        if(camera_controller.showCameraViewAndContinue() == 'q'){
            break;
        }

        /*
        if(frame_snapshot){
            std::cout << "frame snapshot: " << frame_number << std::endl;
            getchar();
        }
        */

        //cvReleaseImage(&left_edge_image);
        //cvReleaseImage(&right_edge_image);
        //cvReleaseImage(&grey_img_left);
        //cvReleaseImage(&grey_img_right);
        frame_number++;
    }

    std::vector<float> params = match_object->getSuperQuadric().getParams();
    for(unsigned i = 0; i < params.size(); i++){
        std::cout << params[i] << " ";
    }
    std::cout << std::endl;

    ObjectShapeVerifier shape_verifier;
    SuperQuadric sq;// = shape_verifier.fitSilhouette(img_pairs, img_features, match_object);

    params = sq.getParams();
    for(unsigned i = 0; i < params.size(); i++){
        std::cout << params[i] << " ";
    }
    std::cout << std::endl;

    delete match_object;
}

void MainController::mouseHandler(int event, int x, int y, int left_or_right){
    x = x/1.5;
    y = y/1.5;

    static bool got_left, got_right;
    static feature left_feature, right_feature;

    if(got_left && left_or_right == 0){ return; }
    if(got_right && left_or_right == 1){ return; }

    if(left_or_right == 0){
        float closest_dist = FLT_MAX;
        for(unsigned i = 0; i < left_sift_features.size(); i++){
            float d = (left_sift_features[i].x-x)*(left_sift_features[i].x-x) +
                      (left_sift_features[i].y-y)*(left_sift_features[i].y-y);
            if(d < closest_dist){
                closest_dist = d;
                left_feature = left_sift_features[i];
            }
        }
        got_left = true;
    }

    if(left_or_right == 1){
        float closest_dist = FLT_MAX;
        for(unsigned i = 0; i < right_sift_features.size(); i++){
            float d = (right_sift_features[i].x-x)*(right_sift_features[i].x-x) +
                      (right_sift_features[i].y-y)*(right_sift_features[i].y-y);
            if(d < closest_dist){
                closest_dist = d;
                right_feature = right_sift_features[i];
            }
        }
        got_right = true;
    }

    if(got_left && got_right){
        float fd = sqrtf(descr_dist_sq(&left_feature, &right_feature));
        float od = Geometry::minAngleDistance(left_feature.ori, right_feature.ori)*180.0f/M_PI;
        std::cout << fd << " " << od << std::endl;
        got_left = got_right = false;
/*
        CvPoint lpos = cvPoint(left_feature.x, left_feature.y);
        cvCircle(img_left, lpos, 3.0, CV_RGB(255, 0, 0), 1.0);
        cvShowImage("LeftEye", img_left);

        CvPoint rpos = cvPoint(right_feature.x, right_feature.y);
        cvCircle(img_right, rpos, 3.0, CV_RGB(255, 0, 0), 1.0);
        cvShowImage("RightEye", img_right);


        cvWaitKey(5);
        */
    }
}

void MainController::loadCamera(bool live_camera, std::string video_path){
    if (live_camera) {
        cam = Camera::getLiveCamera();
    }
    else {
        cam = Camera::getPlaybackCamera(video_path);
    }

    cam_width = cam->getCameraImageWidth();
    cam_height = cam->getCameraImageHeight();

    Settings::instance().addSetting("camera", "res_width", (int) cam_width);
    Settings::instance().addSetting("camera", "res_height", (int) cam_height);
}

void MainController::loadSIFTGenerator(bool live_camera, std::string video_path){
    if (live_camera) {
        sift_generator = new SIFTGenerator(LOCAL_COMPUTE, true, video_path + "sift_file.dat",
                                           cam_width, cam_height);
    }
    else {
        sift_generator = new SIFTGenerator(LOCAL_FILE_READ, false, video_path + "sift_file.dat",
                                           cam_width, cam_height);
    }
}

bool MainController::getNewFrame(bool generate_sift_features){
    // Get the image buffers from the remote camera
    if(!paused){
        if (!cam->getImage(left_cam_buffer, right_cam_buffer)) {
            return false;
        }
    }

    img_left = Common::imageFromBuffer(left_cam_buffer, cam_width, cam_height);
    img_right = Common::imageFromBuffer(right_cam_buffer, cam_width, cam_height);

    if (!paused && generate_sift_features) {
        sift_generator->putNewFrame(left_cam_buffer, right_cam_buffer, cur_frame);
        sift_generator->getNextFrame(left_sift_features, right_sift_features);

        stereo_features = StereoFeatureCorrelation::correlateFeatures(left_sift_features,
                                                                      right_sift_features,
                                                                      cam_height);
    }

    return true;
}

char MainController::showCameraViewAndContinue(void){
    cvShowImage("LeftEye", img_left);
    cvShowImage("RightEye", img_right);

    char key = cvWaitKey(50);
    if (key == 'q') {
        running = false;
    }

    cvReleaseImage(&img_left);
    cvReleaseImage(&img_right);

    return key;
}

void MainController::createViewWindows(void) {
    cvNamedWindow("LeftEye", 1);
    cvMoveWindow("LeftEye", 600, 30);

    cvNamedWindow("RightEye", 1);
    cvMoveWindow("RightEye", 950, 30);
}

void MainController::closeWindow(std::string name){
    // Just because opencv is being stupid here I put it in a loop
    // to make it work.
    for (int i = 0; i < 2; i++) {
        cvDestroyWindow(name.c_str());
        cvWaitKey(5);
    }
}
