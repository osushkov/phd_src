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
#include "Features/SIFTFeature3D.h"

#include <iostream>
#include <fstream>
#include <cassert>

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
            SIFTFeature3D new_feature;
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

bool Object::sceneMatch(const std::vector<SIFTFeature3D> &raw_scene_features,
                        Transform &object_transform,
                        std::vector<unsigned> &matched_scene_features){

    if(object_scene_match == NULL){
        std::cout << "Object Scene Match is NULL!" << std::endl;
        std::ifstream object_shape_file(shape_filename.c_str(), std::ios::in|std::ios::binary);
        std::cout << shape_filename << std::endl;

        unsigned num_frames = 0;
        object_shape_file.read((char*)&num_frames, sizeof(unsigned));
        assert(num_frames < 1000000); // really big number.

        std::cout << "num frames: " << num_frames << std::endl;

        std::vector<SnapshotFrame> object_frames;
        for(unsigned i = 0; i < num_frames; i++){
            SnapshotFrame new_frame = SnapshotFrame::readFrameFeatures(object_shape_file);
            object_frames.push_back(new_frame);
        }

        object_scene_match = new ObjectSceneMatch(object_name, object_frames);

        if(object_superquadric == NULL){
            object_superquadric = new SuperQuadric(1.0f, 1.0f, 3.0f, 3.0f, 3.0f);
            //object_superquadric->load(object_shape_file);
        }

        surface_points.clear();
        unsigned num_surface_points;
        object_shape_file.read((char*)&num_surface_points, sizeof(unsigned));
        for(unsigned i = 0; i < num_surface_points; i++){
            Vector3D new_surface_point;
            object_shape_file.read((char*)&new_surface_point, sizeof(Vector3D));
            surface_points.push_back(new_surface_point);
        }
    }

    Util::Timer timer;
    timer.start();

    std::vector<SIFTFeature3D> filtered_scene_features;
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
    object_scene_match->reset();
    std::cout << "matching" << std::endl;
    bool result = object_scene_match->sceneMatch(filtered_scene_features,
                                                 object_transform,
                                                 matched_scene_features);
    std::cout << "finished matching" << std::endl;
    //last_position = object_transform.shift;
    //have_last_position = result;

    timer.stop();
    //std::cout << "t: " << timer.getNumElapsedSeconds()*1000.0f << std::endl;

    return result;
}

bool Object::sceneMatch(const std::vector<SIFTFeature3D> &scene_features,
                        Transform &object_transform){
    std::vector<unsigned> tmp;
    return sceneMatch(scene_features, object_transform, tmp);
}


void Object::render(Transform transform){
    if(render_superquadric == NULL){
        if(object_superquadric == NULL){
            object_superquadric = new SuperQuadric();

            std::ifstream object_shape_file(shape_filename.c_str(), std::ios::in|std::ios::binary);
            unsigned num_points;
            object_shape_file.read((char*)&num_points, sizeof(unsigned));

            for(unsigned i = 0; i < num_points; i++){
                SIFTFeature3D new_feature;
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
                SIFTFeature3D new_feature;
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

std::vector<Vector3D> Object::getSurfacePoints(void){
    return surface_points;
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
        success &= readString(object_main_file, "surround_sift_path:", surround_sift_path);
        success &= readString(object_main_file, "rotate_video_path:", rotate_video_path);
        success &= readString(object_main_file, "rotate_sift_path:", rotate_sift_path);
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
        success &= writeString(object_main_file, "surround_sift_path:", surround_sift_path);

        success &= writeString(object_main_file, "rotate_video_path:", rotate_video_path);
        success &= writeString(object_main_file, "rotate_sift_path:", rotate_sift_path);
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

    CameraController camera_controller(surround_video_path, surround_sift_path);

    ObjectLearner *object_learner = new ObjectLearner(object_name);
    ObjectSnapshotDB *raw_snapshot_db = new ObjectSnapshotDB(object_name);

    camera_controller.createViewWindows();

    const unsigned sample_frames = 50;
    unsigned buffered_learn_frames = 0;
    const Transform t = Common::cameraToWorldSpaceTransform(0.0f, 25.0f);

    while (camera_controller.isActive() && camera_controller.getNewFrame()) {
        std::vector<float> cur_arm_joints;
        if(camera_controller.haveArmJoints()){
            cur_arm_joints = camera_controller.getArmJoints();
            assert(cur_arm_joints.size() == 6);
        }

        KinectCamera::CorrelatedImage correlated_frame = camera_controller.getCorrelatedFrame();
        for(unsigned i = 0; i < correlated_frame.depth_pixels.size(); i++){
            if(correlated_frame.depth_pixels[i].have_pos){
                correlated_frame.depth_pixels[i].pos = t.mat * correlated_frame.depth_pixels[i].pos + t.shift;
                //Common::cameraPointToWorldSpace(correlated_frame.depth_pixels[i].pos, 0.0f, 25.0f);

            }
        }

        std::vector<SIFTFeature3D> features = camera_controller.getStereoFeatures();
        for(unsigned i = 0; i < features.size(); i++){
            features[i].position = t.mat * features[i].position + t.shift;
            //Common::cameraPointToWorldSpace(features[i].position, 0.0f, 25.0f);
        }
        


        IplImage *frame_img = Common::imageFromCorrelatedFrame(correlated_frame);
        for(unsigned i = 0; i < features.size(); i++){
            draw_features(frame_img, &(features[i].sift_feature), 1);
        }

        object_learner->newFrame(correlated_frame, features, cur_arm_joints);
        buffered_learn_frames++;

        camera_controller.showImageAndContinue(frame_img);
        cvReleaseImage(&frame_img);

        if(buffered_learn_frames >= sample_frames){
            buffered_learn_frames = 0;
            object_learner->learn(raw_snapshot_db);
        }
    }

    if (buffered_learn_frames > 0) {
        buffered_learn_frames = 0;
        object_learner->learn(raw_snapshot_db);
    }

    if(object_name == "arm"){
        snapshot_db = object_learner->generateAugmentedDB(raw_snapshot_db);
        delete raw_snapshot_db;
    }
    else{
        snapshot_db = raw_snapshot_db;
        object_learner->saveSnapshotFrames();
    }

    delete object_learner;

    snapshots_filename = getObjectRootPath() + object_name + "_snapshots.dat";
    snapshot_db->save(snapshots_filename);

    std::cout << "Finished learnObjectSnapshots." << std::endl;
    exit(1);
    return true;
}

bool Object::capturePointCloud(void){
    std::cout << "capturePointCloud - " << object_name << std::endl;

    CameraController camera_controller(surround_video_path, surround_sift_path);
    if(snapshot_db == NULL){
        snapshot_db = new ObjectSnapshotDB(object_name);
        snapshot_db->load(snapshots_filename);
    }

    RawPointCloud point_cloud_generator(object_name);
    point_cloud_generator.generate(snapshot_db, rotate_video_path, rotate_sift_path);

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

    /*
    for(unsigned i = 0; i < point_cloud_frames.size(); i++){
        if(point_cloud_frames[i].object_features.size() > 4){
            reconstruction_manager.submitFeatures(point_cloud_frames[i].object_features,
                                                  point_cloud_frames[i].arm_features);
        }
    }
    */

    std::vector<ModelFrame> model_frames;// = reconstruction_manager.reconstruct();

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
            SIFTFeature3D feature = model_frames[i].features[j];
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
            SIFTFeature3D new_feature;
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
        std::vector<SIFTFeature3D> filtered_frame_features;

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

