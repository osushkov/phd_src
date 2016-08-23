#include "ObjectLearner.h"
#include "../Features/SIFT/sift.h"
#include "../Features/SIFT/imgfeatures.h"
#include "../Features/StereoFeatureCorrelation.h"
#include "../Features/SIFTGenerator.h"
#include "../Settings.h"

#include "SpacioTemporalLocality.h"
#include "../Features/FeatureMemory/FeatureMemory.h"
#include "TraceTracker.h"
#include "FrameHistoryBuffer.h"
#include "../Camera/Camera.h"
#include "../Util/Timer.h"
#include "../Util/PerfStats.h"
#include "../Util/ConvexHull.h"
#include "../Util/Geometry.h"
#include "../Util/ConvexHull.h"
#include "../Arm/ArmForwardKinematics.h"
#include "../Evaluation/MemoryEvaluator.h"

#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>


#define MAX_DMAT_ERROR 1.5f
#define MAX_FEATURE_SCALE 4.0f

ObjectLearner::ObjectLearner(std::string object_name) :
    object_name(object_name),
    current_frame(0),
    outstanding_frames(0),
    accepting_new_frames(true),
    frame_history(300),
    correlated_frame_parser(Settings::instance().getStringValue("general","correlated_features_path")),
    snapshot_db(NULL) {

    //mem_eval.loadBackgroundFeatures("RandomImages/", "RandomImages/file_list.txt");
    //snapshot_db.load("snapshots.dat");
}


ObjectLearner::~ObjectLearner(){
    for (int i = 0; i < 2; i++) {
        cvDestroyWindow("Output");
        cvWaitKey(5);
        cvDestroyWindow("History Left");
        cvWaitKey(5);
        cvDestroyWindow("History Right");
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
    //trace_tracker.printBins();

    // Purge all data connected with the previous learn cycle.
    outstanding_frames = 0;
}


void ObjectLearner::newFrame(const std::vector<unsigned char> &left_frame,
                             const std::vector<unsigned char> &right_frame,
                             const std::vector<StereoFeature> &features,
                             const std::vector<float> &arm_joints){

    if(!accepting_new_frames){ return; }

    frame_history.addToBuffer(left_frame, right_frame, features, arm_joints, current_frame);
    buffered_frames.push_back(current_frame);

    current_frame++;
}


void ObjectLearner::doLearning(void){
    int res_width = Settings::instance().getIntValue("camera", "res_width");
    int res_height = Settings::instance().getIntValue("camera", "res_height");

    for(unsigned i = 0; i < buffered_frames.size(); i++){
        Util::Timer timer("ObjectLearner::doLearning");
        timer.start();

        std::vector<unsigned char> left_frame, right_frame;
        std::vector<StereoFeature> stereo_features;
        frame_history.getHistoryFrame(left_frame, right_frame, stereo_features, buffered_frames[i]);

        IplImage* img_left = Common::imageFromBuffer(left_frame, res_width, res_height);
        IplImage* img_right = Common::imageFromBuffer(right_frame, res_width, res_height);

        // Get rid of feature which have too high a scale and thus are unlikely
        // to be stable since they are likely to depend on segments of the image
        // corresponding to background (not the arm/object)

        //filterFeaturesOnScale(stereo_features, MAX_FEATURE_SCALE); // TODO: need to do this?

        std::vector<float> arm_joints;
        frame_history.getHistoryJoints(arm_joints, buffered_frames[i]);

        if(arm_joints.size() == 6 && object_name != "arm"){
            filterFeaturesOnPosition(stereo_features, arm_joints);
        }

        calculateTrace(stereo_features, buffered_frames[i], img_left, img_right);

        cvReleaseImage(&img_left);
        cvReleaseImage(&img_right);

        timer.stop();
        Util::PerfStats::instance().printStats();
    }

    buffered_frames.clear();
}


void ObjectLearner::filterFeaturesOnScale(std::vector<StereoFeature> &features, float max_scale){
    Util::Timer timer("ObjectLearner::filterFeatures");
    timer.start();

    std::vector<StereoFeature> filtered_features;
    for(unsigned i = 0; i < features.size(); i++){
        if((features[i].feature_left.scl+features[i].feature_left.scl)/2.0f < max_scale){
            filtered_features.push_back(features[i]);
        }
    }
    features = filtered_features;

    timer.stop();
    Util::PerfStats::instance().printStats();
}

void ObjectLearner::filterFeaturesOnPosition(std::vector<StereoFeature> &features,
                                             const std::vector<float> &arm_joints){
    std::vector<StereoFeature> filtered_features;
    for(unsigned i = 0; i < features.size(); i++){
        if(isValidFeaturePosition(features[i].position, arm_joints)){
            filtered_features.push_back(features[i]);
        }
    }
    features = filtered_features;
}


void ObjectLearner::calculateTrace(std::vector<StereoFeature> &features,
                                   unsigned cur_frame,
                                   IplImage* left_frame,
                                   IplImage* right_frame){
    Util::Timer timer("ObjectLearner::calculateTrace");
    timer.start();

    // Add all of the features to the trace tracker.

    std::vector<StereoFeature> arm_features;
    std::vector<StereoFeature> filtered_features;
    std::vector<bool> is_arm_flags;

    if(object_name != "arm"){
        std::vector<FeatureObjectType> types = calculateFeatureTypes(features);
        for(unsigned i = 0; i < features.size(); i++){
            if(types[i] == ARM_FEATURE){
                is_arm_flags.push_back(true);
                arm_features.push_back(features[i]);
            }
            else{
                is_arm_flags.push_back(false);
            }
        }

        filtered_features = features;
/*
        for(unsigned i = 0; i < features.size(); i++){
            bool add = true;
            for(unsigned j = 0; j < arm_features.size(); j++){
                if((features[i].position-arm_features[j].position).length() < 0.5f){
                    add = false;
                    break;
                }
            }
            if(add){
                filtered_features.push_back(features[i]);
            }
        }
        */
    }
    else{
        is_arm_flags = std::vector<bool>(features.size(), false);
        filtered_features = features;
    }

    trace_tracker.submitStereoFeatures(filtered_features, is_arm_flags, cur_frame);

    std::vector<Trace> active_traces = trace_tracker.getActiveTraces();
    renderTraces(active_traces, left_frame, right_frame);

    unsigned frame_start, frame_end;
    std::vector<StereoFeature> features_start, features_end;

    IplImage* left_disp = NULL;
    IplImage* right_disp = NULL;

    // Check to see if this is currently a good time to a correlation between
    // features from different frames.
    bool should_correlate = trace_tracker.shouldCorrelate(frame_start, features_start,
                                                          frame_end, features_end,
                                                          cur_frame);

    if(should_correlate){
        //renderStereoFeatures(features_start, left_frame, right_frame, CV_RGB(0, 0, 255));
        renderStereoFeatures(features_end, left_frame, right_frame, CV_RGB(255, 0, 0));

        //cullFeatures(stable_features, stable_features, arm_features);
        std::vector<StereoFeature> matched_features = features_end;
        filterFeaturesOnScale(matched_features, MAX_FEATURE_SCALE);


        // For the cross correlated features which are part of a moving object, try
        // to classify each feature as either being part of a previously seen object
        // such as the arm, or an unknown feature.
        std::vector<FeatureObjectType> feature_types = calculateFeatureTypes(matched_features);
        std::vector<StereoFeature> new_object_features;
        for(unsigned i = 0; i < matched_features.size(); i++){
            if(object_name == "arm" || feature_types[i] == OBJECT_FEATURE){
                new_object_features.push_back(matched_features[i]);
            }
        }
        matched_features = new_object_features;

        // get rid of detected 'arm feature' which are not close to any others.
        //cullSpacialOutliers(matched_features, feature_types); //TODO

        // Exploit spacial and possibly temporal locality to smooth out errors in classification.
        //localityFilter(matched_features, feature_types);

        // Grab the frame from the end of the trace from history and
        // render on top of it the matched feature with appropriate
        // colours.
        std::vector<unsigned char> left_frame_buf, right_frame_buf;
        std::vector<StereoFeature> stereo_features;
        frame_history.getHistoryFrame(left_frame_buf, right_frame_buf, stereo_features, frame_end);

        left_disp =
            Common::imageFromBuffer(left_frame_buf,
                                    Settings::instance().getIntValue("camera", "res_width"),
                                    Settings::instance().getIntValue("camera", "res_height"));
        right_disp =
            Common::imageFromBuffer(right_frame_buf,
                                    Settings::instance().getIntValue("camera", "res_width"),
                                    Settings::instance().getIntValue("camera", "res_height"));

        //correlated_frame_parser.write(left_disp, right_disp, matched_features,
        //                              feature_types, object_name);

        if(object_name == "arm" || true){
            /*
            std::pair<std::vector<SceneFeature>,std::vector<SceneFeature> > more_features =
                StereoFeatureCorrelation::findAllSceneFeatures(left_disp, right_disp, stereo_features);

            renderSceneFeatures(more_features.second, left_disp, right_disp, CV_RGB(255.0, 0.0, 0.0));
            */
            snapshot_db->addSnapshot(matched_features);
        }

        unsigned total, correct, incorrect;
        //mem_eval.evaluate(*snapshot_db, total, correct, incorrect);
        //std::cout << (float)correct/(float)total << " " << (float)incorrect/(float)total << std::endl;

        renderFeatureTypes(matched_features, feature_types, left_disp);
        cvNamedWindow("Output", 1);
        cvMoveWindow("Output", 10, 10);
        cvShowImage("Output", left_disp);
        cvWaitKey(5);
    }


    cvNamedWindow("History Left", 1);
    cvNamedWindow("History Right", 1);
    cvMoveWindow("History Left", 600, 300);
    cvMoveWindow("History Right", 950, 300);

    cvShowImage("History Left", left_frame);
    cvShowImage("History Right", right_frame);
    cvWaitKey(2);

    if(left_disp != NULL){
        cvReleaseImage(&left_disp);
    }

    if(right_disp != NULL){
        cvReleaseImage(&right_disp);
    }

    timer.stop();
}


void ObjectLearner::localityFilter(std::vector<StereoFeature> &features,
                                   std::vector<FeatureObjectType> &types){
    Util::Timer timer("ObjectLearner::localityFilter");
    timer.start();

    SpacioTemporalLocality locality;
    locality.addFeatures(features, types, 0);

    std::vector<FeatureObjectType> result;
    for(unsigned i = 0; i < features.size(); i++){
        result.push_back(locality.mostProbableType(features[i].position , 0));
    }

    types = result;

    timer.stop();
}


void ObjectLearner::combineFeatureLists(std::vector<StereoFeature> &features,
                                        const std::vector<StereoFeature> &add){
    if(features.size() == 0){
        features = add;
    }
    else {
        for(unsigned i = 0; i < add.size(); i++){
            unsigned closest_index = 0;
            if(!trace_tracker.findClosestFeature(add[i], features, closest_index)){
                features.push_back(add[i]);
            }
        }
    }
}


void ObjectLearner::cullSpacialOutliers(std::vector<StereoFeature> &features,
                                        std::vector<FeatureObjectType> &feature_types){

    Util::Timer timer("ObjectLearner::spacialOutliers");
    timer.start();

    std::vector<StereoFeature> result_features;
    std::vector<FeatureObjectType> result_types;

    for(unsigned i = 0; i < features.size(); i++){
        for(unsigned j = 0; j < features.size(); j++){
            if(j == i || feature_types[i] != feature_types[j]){
                continue;
            }
            if((features[i].position - features[j].position).length() < 4.0f){
                result_features.push_back(features[i]);
                result_types.push_back(feature_types[i]);
                break;
            }
        }
    }
    features = result_features;
    feature_types = result_types;

    timer.stop();
}

void ObjectLearner::cullFeatures(std::vector<StereoFeature> &target_features,
                                 std::vector<StereoFeature> &existing_object_features,
                                 const std::vector<StereoFeature> &arm_features){

    std::vector<StereoFeature> filtered;
    for (unsigned i = 0; i < target_features.size(); i++) {
        float weight = 0.0f;
        for (unsigned j = 0; j < arm_features.size(); j++) {
            float d = (arm_features[j].position - target_features[i].position).length();
            weight -= Common::normalDistribution(0.0f, 2.0f, d);
        }

        for (unsigned j = 0; j < existing_object_features.size(); j++) {
            float d = (existing_object_features[j].position - target_features[i].position).length();
            weight += Common::normalDistribution(0.0f, 2.0f, d);
        }

        if (weight > 0.0f) {
            filtered.push_back(target_features[i]);
        }
    }
    target_features = filtered;
}


void ObjectLearner::renderTraces(const std::vector<Trace> &traces,
                                 IplImage* left_out,
                                 IplImage* right_out){

    for(unsigned i = 0; i < traces.size(); i++){
        bool is_arm_trace = false;
        for(unsigned j = 0; j < traces[i].nodes.size(); j++){
            if(traces[i].nodes[j].is_arm){
                is_arm_trace = true;
                break;
            }
        }

        for(unsigned j = 0; j < traces[i].nodes.size()-1; j++){
            float left_start_x = traces[i].nodes[j].feature.feature_left.x;
            float left_start_y = traces[i].nodes[j].feature.feature_left.y;
            float left_end_x = traces[i].nodes[j+1].feature.feature_left.x;
            float left_end_y = traces[i].nodes[j+1].feature.feature_left.y;

            CvScalar col = CV_RGB(0, 255, 0);
            if(is_arm_trace > 0.5){
                col = CV_RGB(255, 0, 0);
            }

            cvLine(left_out, cvPoint(left_start_x, left_start_y),
                   cvPoint(left_end_x, left_end_y),
                   col, 1, 8, 0);

            float right_start_x = traces[i].nodes[j].feature.feature_right.x;
            float right_start_y = traces[i].nodes[j].feature.feature_right.y;
            float right_end_x = traces[i].nodes[j+1].feature.feature_right.x;
            float right_end_y = traces[i].nodes[j+1].feature.feature_right.y;

            cvLine(right_out, cvPoint(right_start_x, right_start_y),
                   cvPoint(right_end_x, right_end_y),
                   col, 1, 8, 0);
        }
    }
}

void ObjectLearner::renderSceneFeatures(std::vector<SceneFeature> features,
                                        IplImage *left_frame,
                                        IplImage *right_frame,
                                        CvScalar color){

    for(unsigned i = 0; i < features.size(); i++){
        CvPoint left_pos = cvPoint(features[i].left_pos.x,
                                   features[i].left_pos.y);

        CvPoint right_pos = cvPoint(features[i].right_pos.x,
                                    features[i].right_pos.y);

        cvCircle(left_frame, left_pos, 1, color, 2);
        cvCircle(right_frame, right_pos, 1, color, 2);
    }
}

void ObjectLearner::renderStereoFeatures(std::vector<StereoFeature> features,
                                         IplImage *left_frame,
                                         IplImage *right_frame,
                                         CvScalar color){

    for(unsigned i = 0; i < features.size(); i++){
        CvPoint left_pos = cvPoint(features[i].feature_left.x,
                                   features[i].feature_left.y);

        CvPoint right_pos = cvPoint(features[i].feature_right.x,
                                    features[i].feature_right.y);

        cvCircle(left_frame, left_pos, 1, color, 2);
        cvCircle(right_frame, right_pos, 1, color, 2);
    }
}


std::vector<FeatureObjectType>
ObjectLearner::calculateFeatureTypes(const std::vector<StereoFeature> &features){
    Util::Timer timer("ObjectLearner::calculateTypes");
    timer.start();

    std::vector<FeatureObjectType> types;

    if(object_name == "arm"){
        for(unsigned i = 0; i < features.size(); i++){
            types.push_back(ARM_FEATURE);
        }
    }
    else{
        ObjectSnapshotDB *arm_snapshot_db = ObjectSnapshotDB::getArmSnapshotDB();
        assert(arm_snapshot_db != NULL);

        unsigned num_arm = 0;
        SceneMatchResult matches;// = arm_snapshot_db->matchScene(features, false);
        for(unsigned i = 0; i < features.size(); i++){
            FeatureObjectType type = OBJECT_FEATURE;

            if(matches.matched_scene_features.find(i) != matches.matched_scene_features.end()){
                type = ARM_FEATURE;
                num_arm++;
            }

            types.push_back(type);
        }

        // Find the centre of gravity of the object features.
        Vector3D object_cog(0.0f, 0.0f, 0.0f);
        unsigned num_object_features = 0;
        for(unsigned i = 0; i < features.size(); i++){
            if(types[i] == OBJECT_FEATURE){
                object_cog = object_cog + features[i].position;
                num_object_features++;
            }
        }
        object_cog.scale(1.0f/num_object_features);

        // Find the average distance from the CoG
        float avrg = 0.0f;
        for(unsigned i = 0; i < features.size(); i++){
            if(types[i] == OBJECT_FEATURE){
                avrg += (features[i].position - object_cog).length();
            }
        }
        avrg /= num_object_features;

        // Find the standard deviation of the object feature positions.
        float sd = 0.0f;
        for(unsigned i = 0; i < features.size(); i++){
            if(types[i] == OBJECT_FEATURE){
                float diff = (features[i].position - object_cog).length() - avrg;
                sd += diff*diff;
            }
        }
        sd = sqrtf(sd/num_object_features);

        for(unsigned i = 0; i < features.size(); i++){
            float diff = (features[i].position - object_cog).length() - avrg;
            if(sqrtf(diff*diff) > 2.5f*sd){
                types[i] = ARM_FEATURE;
            }
        }

    }

    timer.stop();
    return types;
}


void ObjectLearner::renderFeatureTypes(const std::vector<StereoFeature> &features,
                                       const std::vector<FeatureObjectType> &types,
                                       IplImage *out){

    for(unsigned i = 0; i < features.size(); i++){
        CvPoint feature_pos = cvPoint(features[i].feature_left.x,
                                      features[i].feature_left.y);

        //if(types[i] == OBJECT_FEATURE){
            cvCircle(out, feature_pos, 1, CV_RGB(0, 255, 0), 2);
        /*}
        else if(types[i] == ARM_FEATURE){
            cvCircle(out, feature_pos, 1, CV_RGB(0, 0, 255), 2);
        }*/
    }
}

bool ObjectLearner::isValidFeaturePosition(const Vector3D &position,
                                           const std::vector<float> &arm_joints){
    ArmForwardKinematics fk;
    std::vector<Vector3D> arm_pos = fk.getArmPoint(arm_joints);

    Vector3D arm_wpos = arm_pos[0]; //Common::armPointToWorldSpace(arm_pos[0]);
    Vector3D feature_wpos = Common::cameraPointToWorldSpace(position, 0.0f, -65.0f, 0.0f);
    Vector3D feature_apos = Common::worldPointToArmSpace(feature_wpos);
    feature_apos.scale(0.1f);

    float d = (arm_wpos-feature_apos).length();
    return d < 15.0f;
}

