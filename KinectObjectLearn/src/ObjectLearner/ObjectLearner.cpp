#include "ObjectLearner.h"
#include "../Features/SIFT/sift.h"
#include "../Features/SIFT/imgfeatures.h"
#include "../Features/SIFTFeature3D.h"
#include "../Settings.h"

#include "TraceTracker.h"
#include "FrameHistoryBuffer.h"
#include "../KinectCamera/KinectCamera.h"
#include "../Util/Timer.h"
#include "../Util/PerfStats.h"
#include "../Util/ConvexHull.h"
#include "../Util/Geometry.h"
#include "../Util/Common.h"
#include "../Util/ConvexHull.h"
#include "../Util/Octree.h"
#include "../Util/Octree.hpp"
#include "../Arm/ArmForwardKinematics.h"
#include "../Match.h"

#include <iostream>
#include <cv.h>
#include <highgui.h>


#define MAX_DMAT_ERROR 1.5f
#define MAX_FEATURE_SCALE 4.0f

static std::string getOutputFilename(){
    std::stringstream str;
    static unsigned num;
    str << "data/output/output" << num++ << ".png";
    return str.str();
}

ObjectLearner::ObjectLearner(std::string object_name) :
    object_name(object_name),
    current_frame(0),
    outstanding_frames(0),
    accepting_new_frames(true),
    frame_history(51),
    snapshot_db(NULL) {

}


ObjectLearner::~ObjectLearner(){
    for (int i = 0; i < 2; i++) {
        cvDestroyWindow("Output");
        cvWaitKey(5);
        cvDestroyWindow("History");
        cvWaitKey(5);
    }
}


void ObjectLearner::learn(ObjectSnapshotDB *output_db){
    assert(output_db != NULL);

    snapshot_db = output_db;
    accepting_new_frames = false;

    // Run the main learner algorithm
    doLearning();

    accepting_new_frames = true;

    // Purge all data connected with the previous learn cycle.
    outstanding_frames = 0;
}

ObjectSnapshotDB* ObjectLearner::generateAugmentedDB(ObjectSnapshotDB* snapshot_db){
    ObjectSnapshotDB* result = new ObjectSnapshotDB(snapshot_db->getObjectName());
    std::vector<ObjectSnapshot*> all_snapshots = snapshot_db->getSnapshots();

    for(unsigned i = 0; i < all_snapshots.size(); i++){
        ObjectSnapshot* augments_snapshot = new ObjectSnapshot(*all_snapshots[i]);
        augmentSnapshot(augments_snapshot, all_snapshots);
        result->addSnapshot(augments_snapshot);
    }

    return result;
}


void ObjectLearner::newFrame(KinectCamera::CorrelatedImage frame,
                             std::vector<SIFTFeature3D> features,
                             std::vector<float> arm_joints){

    if(!accepting_new_frames){ return; }
/*
    IplImage *img = Common::imageFromCorrelatedFrame(frame);
    cvSaveImage(getOutputFilename().c_str(), img);
    cvReleaseImage(&img);
*/
    frame_history.addToBuffer(frame, features, arm_joints, current_frame);
    buffered_frames.push_back(current_frame);

    current_frame++;
}

void ObjectLearner::saveSnapshotFrames(void){
    std::stringstream str;
    str << "data/" << object_name << "_rsnapshots.rsnap";
    snapshot_frames_manager.save(str.str());
}

void ObjectLearner::doLearning(void){
    int res_width = Settings::instance().getIntValue("camera", "res_width");
    int res_height = Settings::instance().getIntValue("camera", "res_height");

    for(unsigned i = 0; i < buffered_frames.size(); i++){
        KinectCamera::CorrelatedImage frame;
        std::vector<SIFTFeature3D> stereo_features;
        frame_history.getHistoryFrame(frame, stereo_features, buffered_frames[i]);

        std::vector<float> arm_joints;
        frame_history.getHistoryJoints(arm_joints, buffered_frames[i]);

        filterFeaturesOnWorldPosition(stereo_features);
        if(arm_joints.size() == 6){
            filterFeaturesOnArmPosition(stereo_features, arm_joints);
        }

        calculateTrace(stereo_features, arm_joints, buffered_frames[i], frame);
    }

    buffered_frames.clear();
}

void ObjectLearner::filterFeaturesOnArmPosition(std::vector<SIFTFeature3D> &features,
                                                const std::vector<float> &arm_joints){
    std::vector<SIFTFeature3D> filtered_features;

    ArmForwardKinematics fk;
    std::vector<Vector3D> arm_pos = fk.getArmPoint(arm_joints);
    Vector3D arm_wpos = Common::armPointToWorldSpace(arm_pos[0]);

    for(unsigned i = 0; i < features.size(); i++){
        if(isValidFeaturePosition(features[i].position, arm_wpos)){
            filtered_features.push_back(features[i]);
        }
    }
    features = filtered_features;
}

void ObjectLearner::filterFeaturesOnWorldPosition(std::vector<SIFTFeature3D> &features){
    std::vector<SIFTFeature3D> filtered_features;
    for(unsigned i = 0; i < features.size(); i++){
        if(Common::isPointInWorkspace(features[i].position)){
            filtered_features.push_back(features[i]);
        }
    }
    features = filtered_features;
}

void ObjectLearner::calculateTrace(std::vector<SIFTFeature3D> &features,
                                   std::vector<float> arm_joints,
                                   unsigned cur_frame,
                                   KinectCamera::CorrelatedImage &frame){
    Util::Timer timer;
    timer.start();

    IplImage* frame_img = Common::imageFromCorrelatedFrame(frame);
    //cvSaveImage(getOutputFilename().c_str(), frame_img);
    // Add all of the features to the trace tracker.
    trace_tracker.submitStereoFeatures(features, arm_joints, cur_frame);

    std::vector<Trace> active_traces = trace_tracker.getActiveTraces();
    renderTraces(active_traces, features, frame_img);

    unsigned frame_start, frame_end;
    std::vector<SIFTFeature3D> features_start, features_end;
    std::vector<Trace> feature_paths;

    IplImage* disp_img = NULL;

    // Check to see if this is currently a good time to a correlation between
    // features from different frames.
    bool should_correlate = trace_tracker.shouldCorrelate(frame_start, features_start,
                                                          frame_end, features_end,
                                                          feature_paths,
                                                          cur_frame);

    if(should_correlate){
         std::vector<SIFTFeature3D> matched_features = features_end;

        // Grab the frame from the end of the trace from history and
        // render on top of it the matched feature with appropriate
        // colours.
        KinectCamera::CorrelatedImage history_frame;
        std::vector<SIFTFeature3D> history_features;
        std::vector<float> snapshot_arm_joints;
        frame_history.getHistoryJoints(snapshot_arm_joints, frame_end);
        frame_history.getHistoryFrame(history_frame, history_features, frame_end);
        filterFeaturesOnArmPosition(history_features, snapshot_arm_joints);

        std::vector<bool> arm_mask;
        
        if(object_name == "arm"){
            arm_mask = std::vector<bool>(history_frame.depth_pixels.size(), false);
        } 
        else{
            arm_mask = calculateArmMask(history_frame, history_features, snapshot_arm_joints);
        }

        SnapshotFrame snapshot_frame;

        for(unsigned i = 0; i < arm_mask.size(); i++){
            if(arm_mask[i]){
                snapshot_frame.arm_pixels.push_back(history_frame.depth_pixels[i]);
                history_frame.depth_pixels[i].color = Vector3D(255.0f, 0.0f, 0.0f);
            }
        }

        std::vector<int> counts(history_frame.width*history_frame.height, 0);

        ArmForwardKinematics afk;
        Vector3D arm_pos = Common::armPointToWorldSpace(afk.getArmPoint(snapshot_arm_joints)[0]);

        std::vector<SIFTFeature3D> object_features;
        for(unsigned i = 0; i < matched_features.size(); i++){
            int xpos = (int)matched_features[i].sift_feature.x;
            int ypos = (int)matched_features[i].sift_feature.y;

            if(!arm_mask[xpos + ypos*history_frame.width]){
                if(history_frame.depth_pixels[xpos + ypos*history_frame.width].have_pos){
                    std::vector<bool> mask = floodfillCorrelatedImage(history_frame, 
                        Vector2D(matched_features[i].sift_feature.x, matched_features[i].sift_feature.y),
                        arm_mask, arm_pos, 20.0f);

                    for(unsigned j = 0; j < mask.size(); j++){
                        counts[j] += (int)mask[j];
                    }
                }
            }
        }

        std::vector<SurfacePoint> surface_points;
        for(unsigned i = 0; i < counts.size(); i++){
            if(counts[i] > 2){
                SurfacePoint surface_point;
                surface_point.color = history_frame.depth_pixels[i].color;
                surface_point.pos = history_frame.depth_pixels[i].pos;
                surface_points.push_back(surface_point);

                snapshot_frame.object_pixels.push_back(history_frame.depth_pixels[i]);
                history_frame.depth_pixels[i].color = Vector3D(0.0f, 255.0f, 0.0f);
            }
        }

        for(unsigned i = 0; i < history_features.size(); i++){
            int xpos = (int)history_features[i].sift_feature.x;
            int ypos = (int)history_features[i].sift_feature.y;
            if(counts[xpos + ypos*history_frame.width] > 2){
                snapshot_frame.object_features.push_back(history_features[i]);
                object_features.push_back(history_features[i]);
            }
            else if(arm_mask[xpos + ypos*history_frame.width]){
                snapshot_frame.arm_features.push_back(history_features[i]);
            }
        }
            
        snapshot_frame.to_camera = afk.getToCameraVector(snapshot_arm_joints);
        snapshot_frames_manager.addFrame(snapshot_frame);

        disp_img = Common::imageFromCorrelatedFrame(history_frame);

        if(object_name == "arm"){           
            ArmForwardKinematics afk;

            ObjectSnapshot *new_snapshot = snapshot_db->createNewSnapshot(snapshot_db->getObjectName(),  
                afk.getToCameraVector(snapshot_arm_joints));

            new_snapshot->addFeatures(matched_features, surface_points);
            snapshot_db->addSnapshot(new_snapshot);
        }

        for(unsigned i = 0; i < history_features.size(); i++){
            int xpos = (int)history_features[i].sift_feature.x;
            int ypos = (int)history_features[i].sift_feature.y;
            if(arm_mask[xpos + ypos*history_frame.width]){
                CvPoint feature_pos = cvPoint((int)history_features[i].sift_feature.x,
                                              (int)history_features[i].sift_feature.y);

                cvCircle(disp_img, feature_pos, 1, CV_RGB(255, 0, 0), 2);
            }
        }

        renderFeatureTypes(object_features, disp_img);
        cvSaveImage(getOutputFilename().c_str(), disp_img);
        cvNamedWindow("Output", 1);
        cvMoveWindow("Output", 10, 10);
        cvShowImage("Output", disp_img);
        cvWaitKey(5);
    }


    cvNamedWindow("History", 1);
    cvMoveWindow("History", 600, 300);
    cvShowImage("History", frame_img);
    cvWaitKey(2);

    if(disp_img != NULL){
        cvReleaseImage(&disp_img);
    }

    cvReleaseImage(&frame_img);

    timer.stop();
}

std::vector<bool> ObjectLearner::floodfillCorrelatedImage(KinectCamera::CorrelatedImage &frame, Vector2D scoord,
                                                          const std::vector<bool> &excluded_region,
                                                          Vector3D centre, float max_dist){


    std::vector<Vector2D> stack;
    stack.push_back(scoord);

    std::vector<bool> result(frame.width*frame.height, false);
    while(stack.size() > 0){
        Vector2D cur_px = stack.back();
        stack.pop_back();

        if(excluded_region[(int)cur_px.x + (int)cur_px.y*frame.width]){
            continue;
        }

        if(frame.depth_pixels[(int)cur_px.x + (int)cur_px.y*frame.width].have_pos &&
           (frame.depth_pixels[(int)cur_px.x + (int)cur_px.y*frame.width].pos - centre).length() > max_dist){
            continue;
        }

        result[(int)cur_px.x + (int)cur_px.y*frame.width] = true;

        std::vector<Vector2D> neighbours;
        for(int yo = -2; yo <= 2; yo++){
            for(int xo = -2; xo <= 2; xo++){
                if(xo != 0 || yo != 0){
                    neighbours.push_back(cur_px + Vector2D(xo, yo));
                }
            }
        }
        /*
        neighbours.push_back(cur_px + Vector2D(0.0f, -1.0f));
        neighbours.push_back(cur_px + Vector2D(0.0f, 1.0f));
        neighbours.push_back(cur_px + Vector2D(-1.0f, 0.0f));
        neighbours.push_back(cur_px + Vector2D(1.0f, 0.0f));
        */

        Vector3D cur_pos = frame.depth_pixels[(int)cur_px.x + (int)cur_px.y*frame.width].pos;

        for(unsigned i = 0; i < neighbours.size(); i++){
            if(neighbours[i].x >= 0.0f && neighbours[i].x < frame.width && 
               neighbours[i].y >= 0.0f && neighbours[i].y < frame.height){

                if(frame.depth_pixels[(int)neighbours[i].x + (int)neighbours[i].y*frame.width].have_pos){
                    Vector3D n_pos = frame.depth_pixels[(int)neighbours[i].x + (int)neighbours[i].y*frame.width].pos;
                    if(n_pos.z > 15.0f && (cur_pos - n_pos).length() < 0.5f &&
                       !result[(int)neighbours[i].x + (int)neighbours[i].y*frame.width]){
                        stack.push_back(neighbours[i]);
                    }
                }
            }
        }
    }

    return result;
}

void ObjectLearner::renderTraces(const std::vector<Trace> &traces,
                                 std::vector<SIFTFeature3D> &features,
                                 IplImage* out){

    for(unsigned i = 0; i < features.size(); i++){
        CvScalar col = CV_RGB(0, 0, 255);
        cvCircle(out, cvPoint((int)features[i].sift_feature.x, (int)features[i].sift_feature.y), 2, col);
    }

    for(unsigned i = 0; i < traces.size(); i++){
        for(unsigned j = 0; j < traces[i].nodes.size()-1; j++){
            float start_x = traces[i].nodes[j].feature.sift_feature.x;
            float start_y = traces[i].nodes[j].feature.sift_feature.y;
            float end_x = traces[i].nodes[j+1].feature.sift_feature.x;
            float end_y = traces[i].nodes[j+1].feature.sift_feature.y;

            CvScalar col = CV_RGB(0, 255, 0);
            cvLine(out, cvPoint((int)start_x, (int)start_y),
                   cvPoint((int)end_x, (int)end_y),
                   col, 1, 8, 0);
        }
    }
}

void ObjectLearner::renderStereoFeatures(std::vector<SIFTFeature3D> features,
                                         IplImage *out,
                                         CvScalar color){

    for(unsigned i = 0; i < features.size(); i++){
        CvPoint pos = cvPoint(features[i].sift_feature.x,
                              features[i].sift_feature.y);

        cvCircle(out, pos, 1, color, 2);
    }
}

std::vector<bool> ObjectLearner::calculateArmMask(const KinectCamera::CorrelatedImage &correlated_img, 
                                                  const std::vector<SIFTFeature3D> &features,
                                                  const std::vector<float> &arm_joints){
    
    std::vector<bool> result(correlated_img.depth_pixels.size(), false);                                                  
    ArmForwardKinematics afk;
    Vector3D to_camera = afk.getToCameraVector(arm_joints);

    std::vector<SceneMatchResult> matches = ObjectSnapshotDB::getArmSnapshotDB()->matchSceneMany(features, to_camera, 5.0f);

    /*
    SceneMatchResult best_match;
    float best_score = 0.0f;
    for(unsigned i = 0; i < matches.size(); i++){
        if(matches[i].match_score > best_score){
            best_score = matches[i].match_score;
            best_match = matches[i];
        }
    }

    matches.clear();
    matches.push_back(best_match);
*/

    for(unsigned i = 0; i < matches.size(); i++){
        Match match_refine;
        MatchData match_refine_data;

        match_refine_data.src_features = matches[i].matched_snapshot->getFeatures();
        match_refine_data.dst_features = features;
        match_refine_data.approx_transform = matches[i].approximate_transform;
        match_refine_data.feature_matches = matches[i].feature_matches;

        float err;
        matches[i].approximate_transform = match_refine.doMatch(match_refine_data, err);
    }

    Octree<SurfacePoint> octree(Vector3D(0.0f, 40.0f, 20.0f), 80.0f, 60.0f, 40.0f, 32, 8);
    for(unsigned j = 0; j < matches.size(); j++){
        std::vector<SurfacePoint> surface_points = matches[j].matched_snapshot->getSurfacePoints();

        for(unsigned i = 0; i < surface_points.size(); i++){
            surface_points[i].pos = 
                matches[j].approximate_transform.mat*(surface_points[i].pos - matches[j].approximate_transform.secondary_shift) + 
                matches[j].approximate_transform.shift;

            octree.insertElement(surface_points[i].pos, surface_points[i]);
        }
    }

    for(unsigned i = 0; i < correlated_img.depth_pixels.size(); i++){
        if(correlated_img.depth_pixels[i].have_pos && Common::isPointInWorkspace(correlated_img.depth_pixels[i].pos)){
            std::vector<SurfacePoint> near_surface_points;
            octree.getElementsInRegion(correlated_img.depth_pixels[i].pos, 0.3f, near_surface_points);
            if(near_surface_points.size() > 0){
                result[i] = true;
            }
        }
    }

    return result;
}


void ObjectLearner::renderFeatureTypes(const std::vector<SIFTFeature3D> &features, IplImage *out){

    for(unsigned i = 0; i < features.size(); i++){
        CvPoint feature_pos = cvPoint((int)features[i].sift_feature.x,
                                      (int)features[i].sift_feature.y);

        //if(types[i] == OBJECT_FEATURE){
            cvCircle(out, feature_pos, 1, CV_RGB(0, 0, 255), 2);
        /*}
        else if(types[i] == ARM_FEATURE){
            cvCircle(out, feature_pos, 1, CV_RGB(0, 0, 255), 2);
        }*/
    }
}

bool ObjectLearner::isValidFeaturePosition(const Vector3D &position, const Vector3D gripper_pos){
    if(object_name == "arm"){
        return (gripper_pos-position).length() < 25.0f;
    }
    else{
        return (gripper_pos-position).length() < 20.0f;
    }
}

void ObjectLearner::augmentSnapshot(ObjectSnapshot *snapshot, std::vector<ObjectSnapshot*> all_snapshots){
    std::vector<ObjectSnapshot*> near_snapshots;

    for(unsigned i = 0; i < all_snapshots.size(); i++){
        if(fabs(acosf(snapshot->getToCamera().dotProduct(all_snapshots[i]->getToCamera()))) < 20.0f*(float)M_PI/180.0f){
            near_snapshots.push_back(all_snapshots[i]);
        }
    }

    Match match_refine;
    for(unsigned i = 0; i < near_snapshots.size(); i++){
        std::vector<SnapshotMatchResult> matches;
        Transform at;

        if(near_snapshots[i]->matchScene(snapshot->getFeatures(), matches, at) >= 10.0f){
            MatchData match_refine_data;

            match_refine_data.src_features = near_snapshots[i]->getFeatures();
            match_refine_data.dst_features = snapshot->getFeatures();
            match_refine_data.approx_transform = at;
            match_refine_data.feature_matches = matches;

            float err;
            at = match_refine.doMatch(match_refine_data, err);

            std::vector<SurfacePoint> all_pixels = near_snapshots[i]->getSurfacePoints();
            for(unsigned j = 0; j < all_pixels.size(); j++){
                all_pixels[j].pos = at.mat*(all_pixels[j].pos - at.secondary_shift) + at.shift;
            }

            snapshot->addSurfacePoints(all_pixels);
        }
    }
}
