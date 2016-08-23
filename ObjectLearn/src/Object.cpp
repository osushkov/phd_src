/*
 * Object.cpp
 *
 *  Created on: 30/10/2009
 *      Author: osushkov
 */

#include "Object.h"
#include "CameraController.h"
#include "ObjectLearner/ObjectLearner.h"
#include "Reconstruction/RawPointCloud.h"
#include "Reconstruction/ReconstructionManager.h"
#include "Reconstruction/CentralisePoints.h"
#include "Reconstruction/SuperQuadric.h"
#include "Reconstruction/SuperQuadricBestFit.h"
#include "Reconstruction/BestFit.h"
#include "Visualisation/SuperQuadricRenderObject.h"
#include "Visualisation/LinkedPointsRenderObject.h"
#include "Util/Timer.h"
#include "Util/Common.h"
#include "Util/Geometry.h"
#include "Features/StereoFeatureCorrelation.h"
#include "SilhouetteMatch/ObjectShapeVerifier.h"

#include <iostream>
#include <fstream>
#include <cassert>
#include <sys/stat.h>
#include <sys/types.h>



Object::Object(std::string name) :
    object_name(name),
    current_build_stage(OBJECT_BS_NONE),
    snapshot_db(NULL),
    have_last_position(false),
    object_superquadric(NULL),
    render_superquadric(NULL),
    cull_by_normals(false),
    object_scene_match(NULL) {

    if(!loadObject()){
        createObject();
    }
}


Object::~Object(){
    if(snapshot_db != NULL){
        delete snapshot_db;
    }

    if(object_superquadric != NULL){
        delete object_superquadric;
    }
}


std::string Object::getObjectName(void) const {
    return object_name;
}


ObjectBuildStage Object::getCurrentBuildStage(void) const {
    return current_build_stage;
}


bool Object::buildFull(void){
    return buildStage(OBJECT_BS_FULL);
}


bool Object::buildStage(ObjectBuildStage stage){
    if(current_build_stage == stage){
        return true;
    }

    ObjectBuildStage next_build_stage = (ObjectBuildStage)((int)current_build_stage+1);
    bool success = false;

    switch(next_build_stage){
    case OBJECT_BS_RAW_VIDEO:
        success = captureVideoData();
        break;
    case OBJECT_BS_SNAPSHOTS:
        success = learnObjectSnapshots();
        break;
    case OBJECT_BS_RAW_POINT_CLOUD:
        success = capturePointCloud();
        break;
    case OBJECT_BS_COHERENT_POINT_CLOUD:
        success = buildPointCloud();
        break;
    case OBJECT_BS_FULL:
        success = fitShapeToPointcloud();
        break;
    default:
        assert(false);
    }

    if(!success){
        std::cerr << "Error: could not complete " << buildStageToString(next_build_stage)
                  << std::endl;
        return false;
    }

    /*
    std::cout << "waiting..." << std::endl;
    getchar();
    exit(1);*/

    current_build_stage = next_build_stage;
    writeMainFile();

    return buildStage(stage);
}


bool Object::examine(void){
	if(current_build_stage != OBJECT_BS_FULL){
		return false;
	}

    std::ifstream object_shape_file(shape_filename.c_str(), std::ios::in|std::ios::binary);

    PointCloudRenderObject *point_cloud = new PointCloudRenderObject(false, Vector3D(1.0f,0.0f,0.0f));
    std::vector<PointCloudPoint> all_points;

    std::vector<ModelFrame> model_frames;

    unsigned num_frames;
    object_shape_file.read((char*)&num_frames, sizeof(unsigned));

    for(unsigned i = 0; i < num_frames; i++){
        ModelFrame new_frame;
        object_shape_file.read((char*)&(new_frame.view_direction), sizeof(Vector3D));

        unsigned num_features;
        object_shape_file.read((char*)&num_features, sizeof(unsigned));

        for(unsigned j = 0; j < num_features; j++){
            StereoFeature new_feature;
            Common::readFeature(object_shape_file, new_feature);
            new_frame.features.push_back(new_feature);

            PointCloudPoint new_point;
            new_point.pos = new_feature.position;
            all_points.push_back(new_point);
        }

        model_frames.push_back(new_frame);
    }
    point_cloud->addPoints(all_points);

    SuperQuadric object_superquadric;
    object_superquadric.load(object_shape_file);

    SceneRenderer::instance().initialise();

    SuperQuadricRenderObject *super_quadric = new SuperQuadricRenderObject(object_superquadric);
    SceneRenderer::instance().addObject(super_quadric);

    SceneRenderer::instance().addObject(point_cloud);

    std::cout << "Press Enter to finish." << std::endl;
    getchar();

    SceneRenderer::instance().clearObjects();
    delete super_quadric;
    delete point_cloud;

    return true;
}

bool Object::sceneMatch(const std::vector<StereoFeature> &raw_scene_features,
                        Transform &object_transform,
                        std::vector<unsigned> &matched_scene_features){

    if(object_scene_match == NULL){
        std::cout << "Object Scene Match is NULL!" << std::endl;
        std::ifstream object_shape_file(shape_filename.c_str(), std::ios::in|std::ios::binary);
        std::vector<ModelFrame> object_frames;

        unsigned num_frames = 0;
        object_shape_file.read((char*)&num_frames, sizeof(unsigned));
        assert(num_frames < 1000000); // really big number.

        for(unsigned i = 0; i < num_frames; i++){
            ModelFrame new_frame;
            object_shape_file.read((char*)&(new_frame.view_direction), sizeof(Vector3D));

            unsigned num_features;
            object_shape_file.read((char*)&num_features, sizeof(unsigned));

            for(unsigned j = 0; j < num_features; j++){
                StereoFeature new_feature;
                Common::readFeature(object_shape_file, new_feature);
                new_frame.features.push_back(new_feature);
            }

            object_frames.push_back(new_frame);
        }

        object_scene_match = new ObjectSceneMatch(object_name, object_frames);

        if(object_superquadric == NULL){
            object_superquadric = new SuperQuadric();
            object_superquadric->load(object_shape_file);
        }
    }

    Util::Timer timer;
    timer.start();

    std::vector<StereoFeature> filtered_scene_features;
    if(have_last_position && false){
        for(unsigned i = 0; i < raw_scene_features.size(); i++){
            if((raw_scene_features[i].position-last_position).length() < 15.0f){
                filtered_scene_features.push_back(raw_scene_features[i]);
            }
        }
    }
    else{
        filtered_scene_features = raw_scene_features;
    }

    //object_scene_match->reset(); // TODO: make option.
    bool result = object_scene_match->sceneMatch(filtered_scene_features,
                                                 object_transform,
                                                 matched_scene_features);
    //last_position = object_transform.shift;
    //have_last_position = result;

    timer.stop();
    //std::cout << "t: " << timer.getNumElapsedSeconds()*1000.0f << std::endl;

    return result;
}

bool Object::sceneMatch(const std::vector<StereoFeature> &scene_features,
                        Transform &object_transform){
    std::vector<unsigned> tmp;
    return sceneMatch(scene_features, object_transform, tmp);
}


void Object::render(Transform transform){
    if(render_superquadric == NULL){
        if(object_superquadric == NULL){
            std::cout << "there" << std::endl;
            object_superquadric = new SuperQuadric();

            std::ifstream object_shape_file(shape_filename.c_str(), std::ios::in|std::ios::binary);
            unsigned num_points;
            object_shape_file.read((char*)&num_points, sizeof(unsigned));

            for(unsigned i = 0; i < num_points; i++){
                StereoFeature new_feature;
                Common::readFeature(object_shape_file, new_feature);
            }

            object_superquadric->load(object_shape_file);
        }
        render_superquadric = new SuperQuadricRenderObject(*object_superquadric);
    }

    assert(render_superquadric != NULL);
    if(!SceneRenderer::instance().haveObject(render_superquadric->getId())){
        SceneRenderer::instance().addObject(render_superquadric);
    }

    render_superquadric->setTransform(transform);
}

std::vector<ModelFrame> Object::getModelFrames(void){
    std::vector<ModelFrame> result;

    if(current_build_stage >= OBJECT_BS_COHERENT_POINT_CLOUD){
        std::ifstream point_cloud_file(coherent_point_cloud_filename.c_str(), std::ios::binary);

        unsigned num_frames;
        point_cloud_file.read((char*)&num_frames, sizeof(unsigned));

        for(unsigned i = 0; i < num_frames; i++){
            ModelFrame new_frame;
            point_cloud_file.read((char*)&(new_frame.view_direction), sizeof(Vector3D));

            unsigned num_features;
            point_cloud_file.read((char*)&num_features, sizeof(unsigned));

            for(unsigned j = 0; j < num_features; j++){
                StereoFeature new_feature;
                Common::readFeature(point_cloud_file, new_feature);
                new_frame.features.push_back(new_feature);
            }
            result.push_back(new_frame);
        }
    }
    return result;
}

Mesh Object::getObjectMesh(void){
    return object_superquadric->convertToMesh((float)M_PI/5.0f);
}

SuperQuadric Object::getSuperQuadric(void){
    return *object_superquadric;
}

bool Object::loadObject(void){
    if(!parseMainFile()){
        return false;
    }

    std::cout << "Loaded object: " << object_name << std::endl;
    return true;
}


bool Object::parseMainFile(void){
    std::string object_main_filename = getObjectRootPath() + "object_main.dat";
    std::ifstream object_main_file(object_main_filename.c_str(), std::ios::in);
    std::string tmp;

    // Check to see if the main file exists, if not then return false, couldnt load
    // the object.
    if(!object_main_file.good() || object_main_file.bad() || object_main_file.eof()){
        return false;
    }

    // Check that the file references the correct object name.
    object_main_file >> tmp;
    if(tmp != object_name){
        std::cerr << "Error: object main file references incorrect object name: "
                  << tmp << std::endl;
        return false;
    }

    int cbs;
    object_main_file >> tmp;
    if(tmp != "current_build_stage:"){
        std::cerr << "Error: object main file references incorrect object name: "
                  << tmp << std::endl;
        return false;
    }
    object_main_file >> cbs;
    current_build_stage = (ObjectBuildStage)cbs;
    std::cout << "Current object build stage: "
              << buildStageToString(current_build_stage) << std::endl;


    bool success = true;
    if(current_build_stage >= OBJECT_BS_RAW_VIDEO){
        success &= readString(object_main_file, "surround_video_path:", surround_video_path);
        success &= readString(object_main_file, "rotate_video_path:", rotate_video_path);
    }
    if(current_build_stage >= OBJECT_BS_SNAPSHOTS){
        success &= readString(object_main_file, "snapshots_filename:", snapshots_filename);
    }
    if(current_build_stage >= OBJECT_BS_RAW_POINT_CLOUD){
        success &= readString(object_main_file,
                             "raw_point_cloud_filename:",
                              raw_point_cloud_filename);
    }
    if(current_build_stage >= OBJECT_BS_COHERENT_POINT_CLOUD){
        success &= readString(object_main_file,
                             "coherent_point_cloud_filename:",
                              coherent_point_cloud_filename);
    }
    if(current_build_stage >= OBJECT_BS_FULL){
        success &= readString(object_main_file, "shape_filename:", shape_filename);
    }

    if(!success){
        std::cerr << "Error: incorrect object main file format." << std::endl;
        return false;
    }

    return true;
}


bool Object::writeMainFile(void) {
    std::string object_main_filename = getObjectRootPath() + "object_main.dat";
    std::ofstream object_main_file(object_main_filename.c_str(), std::ios::out);
    std::string tmp;

    std::cout << "Writing" << std::endl;
    // Check to see if the main file exists, if not then return false, couldnt load
    // the object.
    if (!object_main_file.good() || object_main_file.bad() || object_main_file.eof()) {
        return false;
    }

    object_main_file << object_name << std::endl;
    object_main_file << "current_build_stage: " << (int)current_build_stage << std::endl;

    bool success = true;
    if (current_build_stage >= OBJECT_BS_RAW_VIDEO) {
        success &= writeString(object_main_file, "surround_video_path:", surround_video_path);
        success &= writeString(object_main_file, "rotate_video_path:", rotate_video_path);
    }
    if (current_build_stage >= OBJECT_BS_SNAPSHOTS) {
        success &= writeString(object_main_file, "snapshots_filename:", snapshots_filename);
    }
    if (current_build_stage >= OBJECT_BS_RAW_POINT_CLOUD) {
        success &= writeString(object_main_file, "raw_point_cloud_filename:",
                               raw_point_cloud_filename);
    }
    if (current_build_stage >= OBJECT_BS_COHERENT_POINT_CLOUD) {
        success &= writeString(object_main_file, "coherent_point_cloud_filename:",
                               coherent_point_cloud_filename);
    }
    if (current_build_stage >= OBJECT_BS_FULL) {
        success &= writeString(object_main_file, "shape_filename:", shape_filename);
    }

    std::cout << "done Writing" << std::endl;
    if (!success) {
        std::cerr << "Error: could not write object main file." << std::endl;
        return false;
    }

    return true;
}


bool Object::createObject(void){
    std::cout << "Creating object: " << object_name << std::endl;
    if(mkdir(getObjectRootPath().c_str(), 0777) == -1){
        std::cerr << "Could not create object directory: " << getObjectRootPath() << std::endl;
        return false;
    }

    std::string object_main_filename = getObjectRootPath() + "object_main.dat";
    std::ofstream object_main_file(object_main_filename.c_str(), std::ios::out);

    object_main_file << object_name << std::endl;
    object_main_file << (int)current_build_stage << std::endl;

    return true;
}


bool Object::captureVideoData(void){

    return true;
}

bool Object::learnObjectSnapshots(void){
    std::cout << "learnObjectSnapshots - " << object_name << std::endl;

    CameraController camera_controller(false, surround_video_path, true, false);

    ObjectLearner *object_learner = new ObjectLearner(object_name);
    snapshot_db = new ObjectSnapshotDB(object_name);

    camera_controller.createViewWindows();

    const unsigned sample_frames = 100;
    unsigned buffered_learn_frames = 0;
    while (camera_controller.isActive() && camera_controller.getNewFrame()) {
        if(buffered_learn_frames >= sample_frames){
            buffered_learn_frames = 0;
            object_learner->learn(snapshot_db);
        }

        std::vector<float> cur_arm_joints;
        if(camera_controller.haveArmJoints()){
            cur_arm_joints = camera_controller.getArmJoints();
            assert(cur_arm_joints.size() == 6);
        }

        std::vector<unsigned char> left_cam_buffer = camera_controller.getLeftCamBuffer();
        std::vector<unsigned char> right_cam_buffer = camera_controller.getRightCamBuffer();
        std::vector<StereoFeature> features = camera_controller.getStereoFeatures();

        object_learner->newFrame(left_cam_buffer, right_cam_buffer, features, cur_arm_joints);
        buffered_learn_frames++;

        camera_controller.showCameraViewAndContinue();
    }

    if (buffered_learn_frames > 0) {
        buffered_learn_frames = 0;
        object_learner->learn(snapshot_db);
    }

    delete object_learner;

    snapshots_filename = getObjectRootPath() + object_name + "_snapshots.dat";
    snapshot_db->save(snapshots_filename);

    std::cout << "Finished learnObjectSnapshots." << std::endl;
    return true;
}

bool Object::capturePointCloud(void){
    std::cout << "capturePointCloud - " << object_name << std::endl;

    CameraController camera_controller(false, surround_video_path, true, false);
    if(snapshot_db == NULL){
        snapshot_db = new ObjectSnapshotDB(object_name);
        snapshot_db->load(snapshots_filename);
    }

    RawPointCloud point_cloud_generator(object_name);
    point_cloud_generator.generate(snapshot_db, rotate_video_path);

    raw_point_cloud_filename = getObjectRootPath() + object_name + "_raw_point_cloud.dat";
    point_cloud_generator.save(raw_point_cloud_filename);

    std::cout << "Finished capturePointCloud." << std::endl;
    return true;
}

bool Object::buildPointCloud(void){
    std::cout << "buildPointCloud - " << object_name << std::endl;

    RawPointCloud point_cloud_generator(object_name);
    if(!point_cloud_generator.load(raw_point_cloud_filename)){
        return false;
    }

    ReconstructionManager reconstruction_manager(object_name);

    std::vector<RawPointCloudFrame> point_cloud_frames =
        point_cloud_generator.getPointCloudFrames();
    for(unsigned i = 0; i < point_cloud_frames.size(); i++){
        if(point_cloud_frames[i].object_features.size() > 4){
            reconstruction_manager.submitFeatures(point_cloud_frames[i].object_features,
                                                  point_cloud_frames[i].arm_features);
        }
    }

    std::vector<ModelFrame> model_frames = reconstruction_manager.reconstruct();

    coherent_point_cloud_filename = getObjectRootPath() + object_name + "_coherent_point_cloud.dat";
    std::ofstream object_main_file(coherent_point_cloud_filename.c_str(),
                                   std::ios::out|std::ios::binary);

    unsigned num_frames = model_frames.size();
    object_main_file.write((char*)&num_frames, sizeof(unsigned));

    for(unsigned i = 0; i < model_frames.size(); i++){
        object_main_file.write((char*)&(model_frames[i].view_direction), sizeof(Vector3D));

        unsigned num_points = model_frames[i].features.size();
        object_main_file.write((char*)&num_points, sizeof(unsigned));

        for(unsigned j = 0; j < model_frames[i].features.size(); j++){
            Common::writeFeature(object_main_file, model_frames[i].features[j]);
        }
    }

    std::cout << "Finished buildPointCloud." << std::endl;

    SceneRenderer::instance().initialise();
    PointCloudRenderObject *point_cloud = new PointCloudRenderObject();
    SceneRenderer::instance().addObject(point_cloud);

    std::vector<PointCloudPoint> all_points;
    for(unsigned i = 0; i < model_frames.size(); i++){
        for(unsigned j = 0; j < model_frames[i].features.size(); j++){
            PointCloudPoint new_point;
            new_point.pos = model_frames[i].features[j].position;
            all_points.push_back(new_point);
        }
    }

    point_cloud->addPoints(all_points);

    return true;
}

bool Object::fitShapeToPointcloud(void){
    std::cout << "fitShapeToPointcloud: starting" << std::endl;
	std::ifstream object_main_file(coherent_point_cloud_filename.c_str(),
                                   std::ios::in|std::ios::binary);

    std::vector<ModelFrame> model_frames = readModelFrames(object_main_file);
    std::vector<Vector3D> feature_positions = getFeaturePositions(model_frames);
    Vector3D cog = Geometry::centreOfMass(feature_positions);

    shiftToCOG(feature_positions, model_frames, cog);
    std::pair<float,float> distribution = calculatePositionsDistribution(feature_positions);

    std::cout << "fitShapeToPointcloud: filtering features by position" << std::endl;
    float avrg_dist = distribution.first;
    float sd = distribution.second;

    std::cout << "fps: " << feature_positions.size() << std::endl;
    filterFeaturePositionsByDistance(feature_positions, avrg_dist, 3.0f*sd);
    std::cout << "fps: " << feature_positions.size() << std::endl;

    filterModelFramesByDistance(model_frames, avrg_dist, 3.0f*sd);

    std::cout << "fitShapeToPointcloud: centralising points" << std::endl;
    centralisePoints(feature_positions, model_frames);

    SceneRenderer::instance().initialise();

    if(object_scene_match != NULL){
        delete object_scene_match;
    }
    object_scene_match = new ObjectSceneMatch(object_name, model_frames);
/*
    PointCloudRenderObject *point_cloud = new PointCloudRenderObject();
    SceneRenderer::instance().addObject(point_cloud);

    std::vector<PointCloudPoint> all_points;
    for(unsigned i = 0; i < feature_positions.size(); i++){
        PointCloudPoint new_point;
        new_point.pos = feature_positions[i];// + Vector3D(0.0f, 0.0f, 10.0f);
        all_points.push_back(new_point);
    }

    //SuperQuadric sq(0.1, 0.1, 2.15, 3.6, 5.25);
    //SuperQuadricRenderObject *render_superquadric = new SuperQuadricRenderObject(sq);
    //SceneRenderer::instance().addObject(render_superquadric);


    point_cloud->addPoints(all_points);
    getchar();
    */

    ObjectShapeVerifier silhouette_verifier;
    std::pair<std::vector<SceneFeature>,std::vector<SceneFeature> > scene_features =
        initialiseSilhouetteVerifier(silhouette_verifier);

    std::vector<Vector3D> scene_feature_positions0, scene_feature_positions1;
    for(unsigned i = 0; i < scene_features.first.size(); i++){
        scene_feature_positions0.push_back(scene_features.first[i].position);
    }
    for(unsigned i = 0; i < scene_features.second.size(); i++){
        scene_feature_positions1.push_back(scene_features.second[i].position);
    }

    SuperQuadricBestFit sq_fit(silhouette_verifier);
    std::vector<std::pair<SuperQuadricFit,float> > shape_hypotheses;
    shape_hypotheses = sq_fit.calculateShapeHypotheses(feature_positions, scene_feature_positions0, scene_feature_positions1);

    SceneRenderer::instance().clearObjects();
    for(unsigned i = 0; i < shape_hypotheses.size(); i++){
        SuperQuadric sq(shape_hypotheses[i].first.e1,
                        shape_hypotheses[i].first.e2,
                        shape_hypotheses[i].first.A,
                        shape_hypotheses[i].first.B,
                        shape_hypotheses[i].first.C,
                        shape_hypotheses[i].first.xt,
                        shape_hypotheses[i].first.yt);
        std::cout << shape_hypotheses[i].first.A << " " << shape_hypotheses[i].first.B << " " << shape_hypotheses[i].first.C << std::endl;
        std::cout << shape_hypotheses[i].first.xt << " " << shape_hypotheses[i].first.yt << std::endl;

        SuperQuadricRenderObject *render_superquadric = new SuperQuadricRenderObject(sq);
        SceneRenderer::instance().addObject(render_superquadric);

        getchar();
        SceneRenderer::instance().removeObject(render_superquadric->getId());
        delete render_superquadric;
    }


    std::cout << "fitShapeToPointcloud: writing out data" << std::endl;
    shape_filename = getObjectRootPath() + object_name + "_shape.dat";
    std::ofstream object_shape_file(shape_filename.c_str(), std::ios::out|std::ios::binary);

    unsigned num_frames = model_frames.size();
    object_shape_file.write((char*) &num_frames, sizeof(unsigned));

    for (unsigned i = 0; i < model_frames.size(); i++) {
        model_frames[i].view_direction = shape_hypotheses[0].first.transform.mat*model_frames[i].view_direction;
        object_shape_file.write((char*) &(model_frames[i].view_direction), sizeof(Vector3D));

        unsigned num_points = model_frames[i].features.size();
        object_shape_file.write((char*) &num_points, sizeof(unsigned));

        for (unsigned j = 0; j < model_frames[i].features.size(); j++) {
            model_frames[i].features[j].position =
                shape_hypotheses[0].first.transform.mat*model_frames[i].features[j].position - shape_hypotheses[0].first.transform.shift;

            Common::writeFeature(object_shape_file, model_frames[i].features[j]);
        }
    }

    object_superquadric = new SuperQuadric(shape_hypotheses[0].first.e1,
                                           shape_hypotheses[0].first.e2,
                                           shape_hypotheses[0].first.A,
                                           shape_hypotheses[0].first.B,
                                           shape_hypotheses[0].first.C,
                                           shape_hypotheses[0].first.xt,
                                           shape_hypotheses[0].first.yt);
    object_superquadric->save(object_shape_file);
    return true;
}

std::vector<Vector3D>
Object::normaliseFeaturePositions(const std::vector<Vector3D> &feature_positions, SuperQuadricFit fit){
    std::vector<Vector3D> result;
    for(unsigned i = 0; i < feature_positions.size(); i++){
        Vector3D new_pos;
        new_pos = fit.transform.mat*feature_positions[i] - fit.transform.shift;
        result.push_back(new_pos);
    }
    return result;
}

std::vector<ModelFrame>
Object::normaliseModelFrames(const std::vector<ModelFrame> &model_frames, SuperQuadricFit fit){
    std::vector<ModelFrame> result;

    for(unsigned i = 0; i < model_frames.size(); i++){
        ModelFrame cur_frame;
        cur_frame.view_direction = model_frames[i].view_direction;

        for(unsigned j = 0; j < model_frames[i].features.size(); j++){
            StereoFeature feature = model_frames[i].features[j];
            feature.position = fit.transform.mat*feature.position - fit.transform.shift;
            cur_frame.features.push_back(feature);
        }

        result.push_back(cur_frame);
    }

    return result;
}

std::vector<ModelFrame> Object::readModelFrames(std::istream &in_stream){
    std::vector<ModelFrame> model_frames;

    unsigned num_frames;
    in_stream.read((char*)&num_frames, sizeof(unsigned));

    for(unsigned i = 0; i < num_frames; i++){
        ModelFrame new_frame;
        in_stream.read((char*)&(new_frame.view_direction), sizeof(Vector3D));

        unsigned num_features;
        in_stream.read((char*)&num_features, sizeof(unsigned));

        for(unsigned j = 0; j < num_features; j++){
            StereoFeature new_feature;
            Common::readFeature(in_stream, new_feature);
            new_frame.features.push_back(new_feature);
        }

        model_frames.push_back(new_frame);
    }

    return model_frames;
}

std::vector<Vector3D> Object::getFeaturePositions(const std::vector<ModelFrame> &model_frames){
    std::vector<Vector3D> feature_positions;
    for(unsigned i = 0; i < model_frames.size(); i++){
        for(unsigned j = 0; j < model_frames[i].features.size(); j++){
            feature_positions.push_back(model_frames[i].features[j].position);
        }
    }
    return feature_positions;
}

void Object::shiftToCOG(std::vector<Vector3D> &feature_positions,
                        std::vector<ModelFrame> &model_frames,
                        Vector3D cog){
    for(unsigned i = 0; i < feature_positions.size(); i++){
        feature_positions[i] = feature_positions[i] - cog;
    }

    for(unsigned i = 0; i < model_frames.size(); i++){
        for(unsigned j = 0; j < model_frames[i].features.size(); j++){
            model_frames[i].features[j].position = model_frames[i].features[j].position - cog;
        }
    }
}

std::pair<float,float> Object::calculatePositionsDistribution(const std::vector<Vector3D> &feature_positions){
    float avrg_dist = 0.0f;
    for(unsigned i = 0; i < feature_positions.size(); i++){
        avrg_dist += feature_positions[i].length();
    }
    avrg_dist /= feature_positions.size();

    float sd = 0.0f;
    for(unsigned i = 0; i < feature_positions.size(); i++){
        float diff = feature_positions[i].length() - avrg_dist;
        sd += diff*diff;
    }
    sd = sqrtf(sd/feature_positions.size());

    std::pair<float,float> result(avrg_dist, sd);
    return result;
}

void Object::filterFeaturePositionsByDistance(std::vector<Vector3D> &feature_positions,
                                              float avrg_dist, float threshold){
    std::vector<Vector3D> filtered_positions;
    for(unsigned i = 0; i < feature_positions.size(); i++){
        float diff = feature_positions[i].length() - avrg_dist;
        if(sqrtf(diff*diff) < threshold){
            filtered_positions.push_back(feature_positions[i]);
        }
    }

    feature_positions.clear();
    for(unsigned i = 0; i < filtered_positions.size(); i++){
        if(!moreFeaturesInRegion(feature_positions, filtered_positions[i], 0.15f, 5)){
            feature_positions.push_back(filtered_positions[i]);
        }
    }

    //feature_positions = filtered_positions;
}

void Object::filterModelFramesByDistance(std::vector<ModelFrame> &model_frames,
                                         float avrg_dist, float threshold){
    for(unsigned i = 0; i < model_frames.size(); i++){
        std::vector<StereoFeature> filtered_frame_features;

        for(unsigned j = 0; j < model_frames[i].features.size(); j++){
            float diff = model_frames[i].features[j].position.length() - avrg_dist;
            if(sqrtf(diff*diff) < threshold){
                filtered_frame_features.push_back(model_frames[i].features[j]);
            }
        }

        model_frames[i].features = filtered_frame_features;
    }
}

bool Object::moreFeaturesInRegion(const std::vector<Vector3D> &features, Vector3D rc, float rd, unsigned max){
    unsigned num = 0;
    float d2 = rd*rd;

    for(unsigned i = 0; i < features.size(); i++){
        if((rc-features[i]).length2() < d2){
            num++;
            if(num > max){
                return true;
            }
        }
    }

    return false;
}

void Object::centralisePoints(std::vector<Vector3D> &feature_positions,
                              std::vector<ModelFrame> &model_frames){
    CentralisePoints centralise_points;
    Transform central_transform;

    centralise_points.calculateTransform(feature_positions, central_transform);
    for(unsigned i = 0; i < feature_positions.size(); i++){
        feature_positions[i] = central_transform.mat * feature_positions[i];
    }

    for(unsigned i = 0; i < model_frames.size(); i++){
        for(unsigned j = 0; j < model_frames[i].features.size(); j++){
            model_frames[i].features[j].position =
                central_transform.mat*model_frames[i].features[j].position;
        }
        model_frames[i].view_direction = central_transform.mat*model_frames[i].view_direction;
    }
}

std::pair<std::vector<SceneFeature>,std::vector<SceneFeature> >
Object::initialiseSilhouetteVerifier(ObjectShapeVerifier &verifier){
    std::vector<std::pair<std::string,std::string> > silhouette_files =
            ObjectShapeVerifier::getSilhouetteImages("data/vcan_white_rotate/", 5);
/*
    std::vector<std::pair<std::string,std::string> > silhouette_files2 =
            ObjectShapeVerifier::getSilhouetteImages("data/vcan_white_rotate2/", 30);
    silhouette_files.insert(silhouette_files.end(), silhouette_files2.begin(), silhouette_files2.end());
*/

    std::vector<std::pair<std::string,std::string> > silhouette_files3 =
            ObjectShapeVerifier::getSilhouetteImages("data/vcan_white_rotate2/", 5);
    silhouette_files.insert(silhouette_files.end(), silhouette_files3.begin(), silhouette_files3.end());

    //silhouette_files.push_back(std::pair<std::string,std::string>("data/cereal_env/left_1.png", "data/cereal_env/right_1.png"));
    //silhouette_files.push_back(std::pair<std::string,std::string>("data/cereal_env/left_2.png", "data/cereal_env/right_2.png"));


    std::vector<std::pair<IplImage*,IplImage*> > img_pairs;
    std::vector<std::vector<StereoFeature> > img_features;
    for(unsigned i = 0; i < silhouette_files.size(); i++){
        std::cout << "loading: " << silhouette_files[i].first << " " << silhouette_files[i].second << std::endl;
        IplImage *left_img = cvLoadImage(silhouette_files[i].first.c_str());
        IplImage *right_img = cvLoadImage(silhouette_files[i].second.c_str());

        std::vector<feature> left_features, right_features;
        Common::extractFeatures(left_img, left_features);
        Common::extractFeatures(right_img, right_features);
        assert(left_features.size() > 0);
        assert(right_features.size() > 0);

        // TODO replace the 384
        std::vector<StereoFeature> stereo_features =
                StereoFeatureCorrelation::correlateFeatures(left_features, right_features, 384);
        assert(stereo_features.size() > 0);

        img_pairs.push_back(std::pair<IplImage*,IplImage*>(left_img, right_img));
        img_features.push_back(stereo_features);
    }

    return verifier.initialise(img_pairs, img_features, this);
}

// TODO: this should call out to Settings which should get it from a file.
std::string Object::getObjectRootPath(void) const {
    return "objects/" + object_name + "/";
}


bool Object::readString(std::istream &in, std::string label, std::string &result){
    std::string tmp;
    in >> tmp;
    if(tmp != label){
        std::cerr << "Error: incorrect object file format: " << tmp << std::endl;
        return false;
    }

    in >> result;
    return true;
}

bool Object::writeString(std::ostream &out, std::string label, std::string value){
    out << label << " " << value << std::endl;
    return true;
}

std::string Object::buildStageToString(ObjectBuildStage stage){
    switch(stage){
    case OBJECT_BS_NONE:
        return "None";
    case OBJECT_BS_RAW_VIDEO:
        return "Raw Video";
    case OBJECT_BS_SNAPSHOTS:
        return "Snapshots";
    case OBJECT_BS_RAW_POINT_CLOUD:
        return "Raw Point Cloud";
    case OBJECT_BS_COHERENT_POINT_CLOUD:
        return "Coherent Point Cloud";
    case OBJECT_BS_FULL:
        return "Full";
    default:
        assert(false);
        return "";
    }
}

