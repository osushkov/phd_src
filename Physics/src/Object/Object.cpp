/*
 * Object.cpp
 *
 *  Created on: 30/10/2009
 *      Author: osushkov
 */

#include "Object.h"
#include "../CameraController.h"
#include "../Visualisation/PointCloudRenderObject.h"
#include "../Visualisation/SuperQuadricRenderObject.h"
#include "../Util/Timer.h"

#include <iostream>
#include <fstream>
#include <cassert>
#include <sys/stat.h>
#include <sys/types.h>



Object::Object(std::string name) :
    object_name(name),
    current_build_stage(OBJECT_BS_NONE),
    snapshot_db(NULL),
    object_superquadric(NULL),
    render_superquadric(NULL),
    cull_by_normals(false),
    object_scene_match(NULL),
    transform_is_valid(false) {

    if(!loadObject()){
        assert(false);
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
                        unsigned &num_features_matched){

    if(object_scene_match == NULL){
        std::ifstream object_shape_file(shape_filename.c_str(), std::ios::in|std::ios::binary);
        std::vector<ModelFrame> object_frames;

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

    bool result = object_scene_match->sceneMatch(raw_scene_features, object_transform, num_features_matched);

    timer.stop();
    std::cout << "t: " << timer.getNumElapsedSeconds()*1000.0f << std::endl;

    return result;
}

void Object::resetSceneMatch(void){
    if(object_scene_match != NULL){
        object_scene_match->reset();
    }
}


void Object::render(Transform transform){
    if(render_superquadric == NULL){
        if(object_superquadric == NULL){
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

std::vector<Vector3D> Object::getPrimaryAxes(void){
	if(object_superquadric == NULL){
		std::cerr << "Error: getPrimaryAxes, object shape not loaded" << std::endl;
		return std::vector<Vector3D>();
	}
	else{
		return object_superquadric->getPrimaryAxes();
	}
}

double Object::findGripSize(Vector3D gripperA, Vector3D gripperB){
    return object_superquadric->findGripSize(gripperA, gripperB);
}

Transform Object::getObjectTransform(void){
    return object_transform;
}

void Object::setObjectTransform(Transform new_transform){
    object_transform = new_transform;
    transform_is_valid = true;
}

void Object::invalidateTransform(void){
    transform_is_valid = false;
}

bool Object::isTransformValid(void){
    return transform_is_valid;
}

bool Object::loadObject(void){
    if(!parseMainFile()){
        return false;
    }

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

    if (!success) {
        std::cerr << "Error: could not write object main file." << std::endl;
        return false;
    }

    return true;
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

