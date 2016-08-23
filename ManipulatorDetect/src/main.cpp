
#include <iostream>
#include <cstdio>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include <fstream>
#include <list>

#include "RemoteCamera.h"
#include "PlaybackCamera.h"
#include "SIFT/sift.h"
#include "SIFT/imgfeatures.h"
#include "Util/Common.h"
#include "Util/ParallelServer.h"
#include "Timer.h"
#include "StereoFeatureCorrelation.h"
#include "TraceTracker.h"
#include "FrameHistoryBuffer.h"
#include "TemporalDisparity.h"
#include "FeatureMemory.h"
#include "SIFTGenerator.h"
#include "DepthFeaturePublisher.h"
#include "SpacioTemporalLocality.h"

#define CAM_WIDTH 320
#define CAM_HEIGHT 240


unsigned cur_frame = 0;
TraceTracker trace_tracker;
FrameHistoryBuffer frame_history(100);
FeatureMemory feature_memory;
SpacioTemporalLocality locality;

bool generate_sift_features = false;
bool generate_arm_features = false;
bool add_background_features = false;
std::string vid_data  = "data/vid1/";


std::vector<StereoFeature> correlateFeatures(std::vector<StereoFeature> features_start, 
                                             std::vector<StereoFeature> &features_end,
                                             DisparityMatrix disparity_matrix){

    std::vector<StereoFeature> matching_features, transformed = features_end;

    for(unsigned i = 0; i < transformed.size(); i++){
        matrixMultiply(disparity_matrix, transformed[i].position.x, transformed[i].position.y, 
                       transformed[i].position.z);
    }

    std::vector<StereoFeature> new_features_end;
    for(unsigned i = 0; i < transformed.size(); i++){
        unsigned closest_index = 0;
        if(trace_tracker.findClosestFeature(transformed[i], features_start, closest_index) &&
           Common::vectorDist(features_start[closest_index].position, transformed[i].position) < 1.5f){
            matching_features.push_back(features_end[i]);
        }
        else{
            new_features_end.push_back(features_end[i]);
        }
    }
    features_end = new_features_end;

    return matching_features;
}

std::vector<StereoFeature> correlateFeatures(std::vector<StereoFeature> features_start, 
                                             std::vector<StereoFeature> &features_end,
                                             DisparityMatrix disparity_matrix, float x, float y, float z){

    return correlateFeatures(features_start, features_end, disparity_matrix);

    std::vector<StereoFeature> matching_features, transformed = features_end;

    for(unsigned i = 0; i < transformed.size(); i++){
        transformed[i].position.x -= x;
        transformed[i].position.y -= y;
        transformed[i].position.z -= z;
        matrixMultiply(disparity_matrix, transformed[i].position.x, transformed[i].position.y, 
                       transformed[i].position.z);
        transformed[i].position.x += x;
        transformed[i].position.y += y;
        transformed[i].position.z += z;
    }

    std::vector<StereoFeature> new_features_end;
    for(unsigned i = 0; i < transformed.size(); i++){
        unsigned closest_index = 0;
        if(trace_tracker.findClosestFeature(transformed[i], features_start, closest_index) &&
           Common::vectorDist(features_start[closest_index].position, transformed[i].position) < 1.5f){
            matching_features.push_back(features_end[i]);
        }
        else{
            new_features_end.push_back(features_end[i]);
        }
    }
    features_end = new_features_end;

    return matching_features;
}


float averageDist(const std::vector<StereoFeature> &features_start, const std::vector<StereoFeature> &features_end){
    float sum = 0.0f;
    for(unsigned i = 0; i < features_start.size(); i++){
        sum += Common::vectorDist(features_start[i].position, features_end[i].position);
    }
    return sum/(float)(features_start.size());
}

void combineFeatureLists(std::vector<StereoFeature> &features, std::vector<StereoFeature> add){
    if(features.size() == 0){ features = add; }
    else{
        for(unsigned i = 0; i < add.size(); i++){
            unsigned closest_index = 0;
            if(!trace_tracker.findClosestFeature(add[i], features, closest_index)){
                features.push_back(add[i]);
            }
        }
    }
}


void cullSpacialOutliers(std::vector<StereoFeature> &features){
    std::vector<StereoFeature> result;
    for(unsigned i = 0; i < features.size(); i++){
        for(unsigned j = 0; j < features.size(); j++){
            if(j == i){ continue; }
            if(Common::vectorDist(features[i].position, features[j].position) < 5.0f){ 
                result.push_back(features[i]);
                break;
            }
        }
    }
    
}

bool should_pause = false;
void calculateTrace(std::vector<StereoFeature> &features, unsigned cur_frame, IplImage* left_frame, IplImage* right_frame){
    trace_tracker.submitStereoFeatures(features, cur_frame);
    std::vector<Trace> active_traces = trace_tracker.getActiveTraces();

    for(unsigned i = 0; i < active_traces.size(); i++){
        for(unsigned j = 0; j < active_traces[i].nodes.size()-1; j++){
            float left_x_start = active_traces[i].nodes[j].feature.feature_left.x;
            float left_y_start = active_traces[i].nodes[j].feature.feature_left.y;
            float left_x_end = active_traces[i].nodes[j+1].feature.feature_left.x;
            float left_y_end = active_traces[i].nodes[j+1].feature.feature_left.y;

            cvLine(left_frame, cvPoint(left_x_start, left_y_start),
                   cvPoint(left_x_end, left_y_end),
	               CV_RGB(0, 255, 0), 1, 8, 0);


            float right_x_start = active_traces[i].nodes[j].feature.feature_right.x;
            float right_y_start = active_traces[i].nodes[j].feature.feature_right.y;
            float right_x_end = active_traces[i].nodes[j+1].feature.feature_right.x;
            float right_y_end = active_traces[i].nodes[j+1].feature.feature_right.y;

            cvLine(right_frame, cvPoint(right_x_start, right_y_start),
                   cvPoint(right_x_end, right_y_end),
	               CV_RGB(0, 255, 0), 1, 8, 0);
        }
    }

    unsigned frame_start, frame_end;
    std::vector<StereoFeature> features_start, features_end;

    if(trace_tracker.isNMDMaxima(frame_start, features_start, frame_end, features_end) &&
       averageDist(features_start, features_end) > 2.0f){
        

        for(unsigned i = 0; i < features_start.size(); i++){
            cvCircle(left_frame, cvPoint(features_start[i].feature_left.x, features_start[i].feature_left.y), 1, CV_RGB(0, 0, 255), 2);
            cvCircle(right_frame, cvPoint(features_start[i].feature_right.x, features_start[i].feature_right.y), 1, CV_RGB(0, 0, 255), 2);
        }

        for(unsigned i = 0; i < features_end.size(); i++){
            cvCircle(left_frame, cvPoint(features_end[i].feature_left.x, features_end[i].feature_left.y), 1, CV_RGB(255, 0, 0), 2);
            cvCircle(right_frame, cvPoint(features_end[i].feature_right.x, features_end[i].feature_right.y), 1, CV_RGB(255, 0, 0), 2);
        }

        //should_pause = true;

        float error = 0.0f;
        float x = 0.0f, y = 0.0f, z = 0.0f;
        //DisparityMatrix disparity_matrix = computeOptimalDisparityRotationMatrix(features_start, features_end, error, x, y, z);
        DisparityMatrix disparity_matrix = computeOptimalDisparityMatrix(features_start, features_end, error);
        std::cout << "dmat error: " << error << std::endl;        
        if(error/(float)(features_start.size()) > 1.25f){ return; }

/*
        for(unsigned y = 0; y < 4; y++){
            for(unsigned x = 0; x < 4; x++){
                std::cout << disparity_matrix.mat[y][x] << "\t";
            }
            std::cout << std::endl;
        }
*/
        std::vector<StereoFeature> matched_features, unmatched_features, target_features, tmp;

        unmatched_features = trace_tracker.getFrameFeatures(frame_end);
        combineFeatureLists(unmatched_features, trace_tracker.getFrameFeatures(frame_end+1));
        combineFeatureLists(unmatched_features, trace_tracker.getFrameFeatures(frame_end+2));
        combineFeatureLists(unmatched_features, trace_tracker.getFrameFeatures(frame_end+3));
        combineFeatureLists(unmatched_features, trace_tracker.getFrameFeatures(frame_end-1));
        combineFeatureLists(unmatched_features, trace_tracker.getFrameFeatures(frame_end-2));
        combineFeatureLists(unmatched_features, trace_tracker.getFrameFeatures(frame_end-3));

        tmp = correlateFeatures(trace_tracker.getFrameFeatures(frame_start), 
                                unmatched_features, disparity_matrix, x, y, z);
        while(tmp.size() > 0){ 
            matched_features.push_back(tmp.back()); 
            tmp.pop_back();
        }

        tmp = correlateFeatures(trace_tracker.getFrameFeatures(frame_start-3), 
                                unmatched_features, disparity_matrix, x, y, z);
        while(tmp.size() > 0){ 
            matched_features.push_back(tmp.back()); 
            tmp.pop_back();
        }


        tmp = correlateFeatures(trace_tracker.getFrameFeatures(frame_start-2), 
                                unmatched_features, disparity_matrix, x, y, z);
        while(tmp.size() > 0){ 
            matched_features.push_back(tmp.back()); 
            tmp.pop_back();
        }


        tmp = correlateFeatures(trace_tracker.getFrameFeatures(frame_start-1), 
                                unmatched_features, disparity_matrix, x, y, z);
        while(tmp.size() > 0){ 
            matched_features.push_back(tmp.back()); 
            tmp.pop_back();
        }

        tmp = correlateFeatures(trace_tracker.getFrameFeatures(frame_start+1), 
                                unmatched_features, disparity_matrix, x, y, z);
        while(tmp.size() > 0){ 
            matched_features.push_back(tmp.back()); 
            tmp.pop_back();
        }

        tmp = correlateFeatures(trace_tracker.getFrameFeatures(frame_start+2), 
                                unmatched_features, disparity_matrix, x, y, z);
        while(tmp.size() > 0){ 
            matched_features.push_back(tmp.back()); 
            tmp.pop_back();
        }

        tmp = correlateFeatures(trace_tracker.getFrameFeatures(frame_start+3), 
                                unmatched_features, disparity_matrix, x, y, z);
        while(tmp.size() > 0){ 
            matched_features.push_back(tmp.back()); 
            tmp.pop_back();
        }



        IplImage* disp = cvCreateImage(cvSize(CAM_WIDTH, CAM_HEIGHT),IPL_DEPTH_8U,3);
        std::vector<char> left_frame_buf, right_frame_buf;
        frame_history.getHistoryFrame(left_frame_buf, right_frame_buf, frame_end);
        disp->imageData = (char*)&(left_frame_buf[0]);
        cvNamedWindow("Blah", 1);

        cullSpacialOutliers(matched_features); // get rid of detected 'arm feature' which are not close to any others.
        for(unsigned i = 0; i < matched_features.size(); i++){
            if(feature_memory.classifyFeature(matched_features[i].feature_left, APPROX_NN) != ARM_FEATURE && 
               feature_memory.classifyFeature(matched_features[i].feature_right, APPROX_NN) != ARM_FEATURE){

                //feature_memory.insertFeature(ARM_FEATURE, matched_features[i].feature_left);
                //feature_memory.insertFeature(ARM_FEATURE, matched_features[i].feature_right);
                cvCircle(disp, cvPoint(matched_features[i].feature_left.x, matched_features[i].feature_left.y), 
                         1, CV_RGB(0, 255, 0), 2);
            }
        }
    
        cvShowImage("Blah", disp);        
    }
}


float minDist(const StereoFeature &feature, const std::vector<StereoFeature> &all_features){
    float min_dist = FLT_MAX;
    for(unsigned i = 0; i < all_features.size(); i++){
        float d = (feature.feature_left.x-all_features[i].feature_left.x)*(feature.feature_left.x-all_features[i].feature_left.x) +
                  (feature.feature_left.y-all_features[i].feature_left.y)*(feature.feature_left.y-all_features[i].feature_left.y);
        d = sqrtf(d);
        if(d < min_dist){ min_dist = d; }

        d = (feature.feature_right.x-all_features[i].feature_right.x)*(feature.feature_right.x-all_features[i].feature_right.x) +
            (feature.feature_right.y-all_features[i].feature_right.y)*(feature.feature_right.y-all_features[i].feature_right.y);
        d = sqrtf(d);
        if(d < min_dist){ min_dist = d; }

/*
        float d = dist(feature.position, all_features[i].position);
        if(d < min_dist){ min_dist = d; }*/
    }
    return min_dist;
}

void spaciallyFilter(std::vector<FeatureObjectType> &feature_types, const std::vector<StereoFeature> &stereo_features){
    assert(feature_types.size() == stereo_features.size());
    unsigned num_changed = 0;

    for(unsigned i = 0; i < stereo_features.size(); i++){
        locality.addFeature(stereo_features[i], feature_types[i], cur_frame);
    }

    for(unsigned i = 0; i < stereo_features.size(); i++){
        FeatureObjectType probable_type = locality.mostProbableType(stereo_features[i].position, cur_frame);
        //if(feature_types[i] == BACKGROUND_FEATURE){ feature_types[i] = probable_type; num_changed++; }
        feature_types[i] = probable_type; num_changed++;
    }
    

/*    float sigma = 3.0f;
    std::vector<FeatureObjectType> new_feature_types;

    for(unsigned i = 0; i < feature_types.size(); i++){
        if(feature_types[i] == BACKGROUND_FEATURE){
            float aweight = 0.0f, bweight = 0.0f;
            for(unsigned j = 0; j < feature_types.size(); j++){
                if(feature_types[j] == ARM_FEATURE){
                    float d = dist(stereo_features[i].position, stereo_features[j].position);
                    aweight += expf(-(d*d)/(2.0f*sigma*sigma));
                }
                else if(feature_types[j] == BACKGROUND_FEATURE){
                    float d = dist(stereo_features[i].position, stereo_features[j].position);
                    bweight += expf(-(d*d)/(2.0f*sigma*sigma));
                }

            }
            if(aweight > 1.0f && aweight > bweight){
                new_feature_types.push_back(ARM_FEATURE);
            }
            else{
                new_feature_types.push_back(feature_types[i]);
            }
        }
        else{
            new_feature_types.push_back(feature_types[i]);
        }
    }

    feature_types = new_feature_types;*/
}

int main(int argc, char **argv){
    if(argc < 2){
        std::cout << "Too few arguments" << std::endl;
        return 1;
    }

    if(argc > 2){ 
        for(int i = 1; i < argc-1; i++){
            std::string operation(argv[i]);
            if(operation == "-sift_gen"){ generate_sift_features = true; }
            else if(operation == "-arm_gen"){ generate_arm_features = true; }
            else if(operation == "-feature_add"){ add_background_features = true; }
        }
    }

    vid_data  = std::string(argv[argc-1]);
 

    Camera *cam = new PlaybackCamera(vid_data);
   
    unsigned cam_width = cam->getCameraImageWidth();
    unsigned cam_height = cam->getCameraImageHeight();

    IplImage* img_left = cvCreateImage(cvSize(cam_width, cam_height),IPL_DEPTH_8U,3);
    IplImage* img_right = cvCreateImage(cvSize(cam_width, cam_height),IPL_DEPTH_8U,3);

    cvNamedWindow("Left Eye", 1);
    cvNamedWindow("Right Eye", 1);

    std::vector<char> left_cam_buffer, right_cam_buffer;
    left_cam_buffer.resize(cam_width*cam_height*3);
    right_cam_buffer.resize(cam_width*cam_height*3);

    std::vector<feature> left_sift_features, right_sift_features;

    std::string sift_file_path = vid_data + std::string("sift_file.dat");
    SIFTGenerator generator(sift_file_path, !generate_sift_features, cam_width, cam_height);
    DepthFeaturePublisher::initialise();


    feature_memory.load("data/feature_memory.dat");

    while(cam->isActive()){
        char key = cvWaitKey(2);		//Need this to update the image windows.
        if(key == 'q'){
            break;
        }

        // Get the image buffers from the remote camera
        cam->getImage(left_cam_buffer, right_cam_buffer);

        // Record the new frame in history.
        frame_history.addToBuffer(left_cam_buffer, right_cam_buffer, cur_frame);

        img_left->imageData = (char*)&(left_cam_buffer[0]);
        img_right->imageData = (char*)&(right_cam_buffer[0]);


        if(generate_sift_features){
            generator.putNewFrame(left_cam_buffer, right_cam_buffer, cur_frame);
            generator.getNextFrame(left_sift_features, right_sift_features);
        }
        else{
            generator.getNextFrame(left_sift_features, right_sift_features);
        }

        if(add_background_features && (cur_frame%5) != 0){ cur_frame++; continue; } // skip some frames

        std::vector<StereoFeature> r, all_arm_features, stereo_features = 
            StereoFeatureCorrelation::correlateFeatures(left_sift_features, right_sift_features);


        if(add_background_features){
            for(unsigned i = 0; i < stereo_features.size(); i++){
                feature_memory.insertFeature(BACKGROUND_FEATURE, stereo_features[i].feature_left);
                feature_memory.insertFeature(BACKGROUND_FEATURE, stereo_features[i].feature_right);
            }
        }

        //DepthFeaturePublisher::submitNewFeatures(stereo_features);

        std::vector<feature> left_features, right_features;

        // work out what type of feature each stereo feature is
       
        std::vector<FeatureObjectType> feature_types;
        /*if(!generate_arm_features && !generate_sift_features && !add_background_features){
            std::vector<feature> all_current_features;
            for(unsigned i = 0; i < stereo_features.size(); i++){
                all_current_features.push_back(stereo_features[i].feature_left);
                all_current_features.push_back(stereo_features[i].feature_right);
            }

            std::vector<FeatureObjectType> tmp_feature_types = feature_memory.classifyFeatures(all_current_features, APPROX_NN);
            assert(tmp_feature_types.size() == 2*stereo_features.size());
            for(unsigned i = 0; i < stereo_features.size(); i++){
                if(tmp_feature_types[i*2] == ARM_FEATURE && tmp_feature_types[i*2 + 1] == ARM_FEATURE){
                    feature_types.push_back(ARM_FEATURE);
                }
                else{
                    feature_types.push_back(BACKGROUND_FEATURE);
                }
            }

            //spaciallyFilter(feature_types, stereo_features);
            assert(feature_types.size() == stereo_features.size());
        }*/

        for(unsigned i = 0; i < stereo_features.size(); i++){
            if(false && !generate_arm_features && !generate_sift_features && !add_background_features && feature_types[i] == ARM_FEATURE){
                cvCircle(img_left, cvPoint(stereo_features[i].feature_left.x, stereo_features[i].feature_left.y), 1, CV_RGB(0, 0, 255), 2.0);
                cvCircle(img_right, cvPoint(stereo_features[i].feature_right.x, stereo_features[i].feature_right.y), 1, CV_RGB(0, 0, 255), 2.0);
                all_arm_features.push_back(stereo_features[i]);
            }
            else{
                cvCircle(img_left, cvPoint(stereo_features[i].feature_left.x, stereo_features[i].feature_left.y), 1, CV_RGB(0, 255, 0), 2.0);
                cvCircle(img_right, cvPoint(stereo_features[i].feature_right.x, stereo_features[i].feature_right.y), 1, CV_RGB(0, 255, 0), 2.0);
                r.push_back(stereo_features[i]);
            }
        }

        stereo_features = r;
        r.clear();


        //if(generate_arm_features && !add_background_features){
            calculateTrace(stereo_features, cur_frame, img_left, img_right);
        //}

        cvShowImage("Left Eye", img_left);
        cvShowImage("Right Eye", img_right);

        if(should_pause){
            key = cvWaitKey();
            should_pause = false;
        }

        cur_frame++;
        //sleep(1);
        //usleep(200000);

    }

    //feature_memory.save("data/feature_memory.dat");


    return 0;
}
