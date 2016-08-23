/*
 * RawPointCloud.cpp
 *
 *  Created on: 01/11/2009
 *      Author: osushkov
 */

#include "RawPointCloud.h"
#include "../CameraController.h"
#include "../Features/FeatureMemory/ObjectSnapshotDB.h"
#include "../Util/Timer.h"

#include <fstream>


RawPointCloud::RawPointCloud(std::string object_name) :
    object_name(object_name){

}


RawPointCloud::~RawPointCloud(){

}


bool RawPointCloud::load(std::string filename){
    std::fstream point_cloud_file(filename.c_str(), std::ios::in | std::ios::binary);

    if(!point_cloud_file.good() || point_cloud_file.eof() || point_cloud_file.bad()){
        return false;
    }

    object_name.clear();

    char tmp;
    while(!point_cloud_file.eof()){
        point_cloud_file.read(&tmp, sizeof(char));
        if(tmp == '\0'){
            break;
        }
        object_name.push_back(tmp);
    }


    unsigned num_frames = 0;
    point_cloud_file.read((char*)&num_frames, sizeof(unsigned));

    for(unsigned i = 0; i < num_frames; i++){
        RawPointCloudFrame new_frame;

        unsigned num_object_features = 0;
        unsigned num_arm_features = 0;

        point_cloud_file.read((char*)&num_object_features, sizeof(unsigned));
        point_cloud_file.read((char*)&num_arm_features, sizeof(unsigned));

        for(unsigned j = 0; j < num_object_features; j++){
            StereoFeature new_feature;
            Common::readFeature(point_cloud_file, new_feature);
            new_frame.object_features.push_back(new_feature);
        }

        for(unsigned j = 0; j < num_arm_features; j++){
            StereoFeature new_feature;
            Common::readFeature(point_cloud_file, new_feature);
            new_frame.arm_features.push_back(new_feature);
        }

        point_cloud_frames.push_back(new_frame);
    }

    return true;
}


bool RawPointCloud::save(std::string filename){
    std::fstream point_cloud_file(filename.c_str(), std::ios::out | std::ios::binary);
    point_cloud_file.write(object_name.c_str(), (object_name.length()+1)*sizeof(char));

    unsigned num_frames = point_cloud_frames.size();
    point_cloud_file.write((char*)&num_frames, sizeof(unsigned));

    std::cout << "Saving " << num_frames << " frames." << std::endl;
    for(unsigned i = 0; i < point_cloud_frames.size(); i++){
        unsigned num_object_features = point_cloud_frames[i].object_features.size();
        unsigned num_arm_features = point_cloud_frames[i].arm_features.size();

        point_cloud_file.write((char*)&num_object_features, sizeof(unsigned));
        point_cloud_file.write((char*)&num_arm_features, sizeof(unsigned));

        for(unsigned j = 0; j < point_cloud_frames[i].object_features.size(); j++){
            Common::writeFeature(point_cloud_file, point_cloud_frames[i].object_features[j]);
        }

        for(unsigned j = 0; j < point_cloud_frames[i].arm_features.size(); j++){
            Common::writeFeature(point_cloud_file, point_cloud_frames[i].arm_features[j]);
        }
    }

    return true;
}


void RawPointCloud::generate(ObjectSnapshotDB *object_snapshot_db, std::string video_path){
    CameraController camera_controller(false, video_path, true, false);
    ObjectSnapshotDB *arm_snapshot_db = ObjectSnapshotDB::getArmSnapshotDB();

    camera_controller.createViewWindows();
    unsigned frame = 0;

    while (camera_controller.isActive()) {
        if (!camera_controller.getNewFrame()) {
            break;
        }

        std::vector<StereoFeature> frame_features = camera_controller.getStereoFeatures();

        Util::Timer timer0;
        timer0.start();
        SceneMatchResult arm_match_result = arm_snapshot_db->matchScene(frame_features, false);
        timer0.stop();

        std::vector<StereoFeature> remaining_features;
        for(unsigned i = 0; i < frame_features.size(); i++){
            if(arm_match_result.matched_scene_features.find(i) ==
               arm_match_result.matched_scene_features.end()){
                remaining_features.push_back(frame_features[i]);
            }
        }

        Util::Timer timer1;
        timer1.start();
        SceneMatchResult object_match_result = object_snapshot_db->matchScene(remaining_features, false);
        timer1.stop();
        /*std::cout << timer0.getNumElapsedSeconds()*1000.0f << " "
                  << timer1.getNumElapsedSeconds()*1000.0f << std::endl;*/

        RawPointCloudFrame new_point_cloud_frame;
        std::set<unsigned>::iterator it;

        IplImage *img_left = camera_controller.getLeftImage();
        IplImage *img_right = camera_controller.getRightImage();


        std::set<unsigned> new_arm_features;
        for(unsigned i = 0; i < remaining_features.size(); i++){

            if(object_match_result.matched_scene_features.find(i) !=
               object_match_result.matched_scene_features.end()){

               std::set<unsigned>::iterator it;
               for(it = arm_match_result.matched_scene_features.begin();
                   it != arm_match_result.matched_scene_features.end();
                   ++it){

                   if((frame_features[*it].position-remaining_features[i].position).length() < 1.0f){
                       new_arm_features.insert(i);
                       break;
                   }
               }
            }
        }


        for(it = new_arm_features.begin(); it != new_arm_features.end(); ++it){
            new_point_cloud_frame.arm_features.push_back(remaining_features[*it]);
        }

        for(it = object_match_result.matched_scene_features.begin();
            it != object_match_result.matched_scene_features.end();
            ++it){

            if(new_arm_features.find(*it) != new_arm_features.end()){ continue; }

            assert(*it >= 0 && *it < remaining_features.size());
            new_point_cloud_frame.object_features.push_back(remaining_features[*it]);
        }

        for(it = arm_match_result.matched_scene_features.begin();
            it != arm_match_result.matched_scene_features.end();
            ++it){

            assert(*it >= 0 && *it < frame_features.size());
            new_point_cloud_frame.arm_features.push_back(frame_features[*it]);
        }

        std::vector<StereoFeature> new_object_features, more_arm_features;
        partitionFeatureSet(new_point_cloud_frame.object_features, 3.0f,
                            new_object_features, more_arm_features);

        new_point_cloud_frame.object_features = new_object_features;
        for(unsigned i = 0; i < more_arm_features.size(); i++){
            new_point_cloud_frame.arm_features.push_back(more_arm_features[i]);
        }

        point_cloud_frames.push_back(new_point_cloud_frame);
        renderDetectedPoints(camera_controller, new_point_cloud_frame);

        frame++;
        camera_controller.showCameraViewAndContinue();
    }
}


std::vector<RawPointCloudFrame> RawPointCloud::getPointCloudFrames(void) const {
    return point_cloud_frames;
}


void RawPointCloud::renderDetectedPoints(CameraController &camera_controller,
                                         RawPointCloudFrame &point_cloud_frame) {
    // Render the features.
    IplImage *img_left = camera_controller.getLeftImage();
    IplImage *img_right = camera_controller.getRightImage();

    for (unsigned i = 0; i < point_cloud_frame.object_features.size(); i++) {
        CvPoint lpos = cvPoint(point_cloud_frame.object_features[i].feature_left.x,
                               point_cloud_frame.object_features[i].feature_left.y);
        CvPoint rpos = cvPoint(point_cloud_frame.object_features[i].feature_right.x,
                               point_cloud_frame.object_features[i].feature_right.y);

        cvCircle(img_left, lpos, 1, CV_RGB(0, 255, 0), 2.0);
        cvCircle(img_right, rpos, 1, CV_RGB(0, 255, 0), 2.0);
    }

    for (unsigned i = 0; i < point_cloud_frame.arm_features.size(); i++) {
        CvPoint lpos = cvPoint(point_cloud_frame.arm_features[i].feature_left.x,
                               point_cloud_frame.arm_features[i].feature_left.y);
        CvPoint rpos = cvPoint(point_cloud_frame.arm_features[i].feature_right.x,
                               point_cloud_frame.arm_features[i].feature_right.y);

        cvCircle(img_left, lpos, 1, CV_RGB(0, 0, 255), 2.0);
        cvCircle(img_right, rpos, 1, CV_RGB(0, 0, 255), 2.0);
    }
}

void RawPointCloud::partitionFeatureSet(const std::vector<StereoFeature> &start_features,
                                        float sd_threshold,
                                        std::vector<StereoFeature> &in_features,
                                        std::vector<StereoFeature> &out_features){

    // Find the centre of gravity of the object features.
    Vector3D cog(0.0f, 0.0f, 0.0f);
    for(unsigned i = 0; i < start_features.size(); i++){
        cog = cog + start_features[i].position;
    }
    cog.scale(1.0f/start_features.size());

    // Find the average distance from the CoG
    float avrg = 0.0f;
    for(unsigned i = 0; i < start_features.size(); i++){
        avrg += (start_features[i].position - cog).length();
    }
    avrg /= start_features.size();

    // Find the standard deviation of the object feature positions.
    float sd = 0.0f;
    for(unsigned i = 0; i < start_features.size(); i++){
        float diff = (start_features[i].position - cog).length() - avrg;
        sd += diff*diff;
    }
    sd = sqrtf(sd/start_features.size());

    for(unsigned i = 0; i < start_features.size(); i++){
        float diff = (start_features[i].position - cog).length() - avrg;
        if(sqrtf(diff*diff) < sd_threshold*sd){
            in_features.push_back(start_features[i]);
        }
        else{
            out_features.push_back(start_features[i]);
        }
    }
}
