/*
 * MemoryEvaluator.cpp
 *
 *  Created on: 23/04/2009
 *      Author: osushkov
 */

#include "MemoryEvaluator.h"
#include "../Features/SIFT/sift.h"
#include "../Features/StereoFeatureCorrelation.h"
#include "../Features/FeatureMemory/ObjectSnapshotDB.h"
#include "../Settings.h"
#include "../Util/Common.h"

#include <cassert>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>


MemoryEvaluator::MemoryEvaluator(std::string root_file, std::string path_prefix){
    std::ifstream in_file(root_file.c_str());

    unsigned num_frames = 0;
    in_file >> num_frames;
    std::cout << "num frames: " << num_frames << std::endl;

    for(unsigned i = 0; i < num_frames; i++){
        std::string left_name, right_name;
        std::string left_mask_name, right_mask_name;

        in_file >> left_name;
        in_file >> right_name;

        in_file >> left_mask_name;
        in_file >> right_mask_name;

        left_name = path_prefix + left_name;
        right_name = path_prefix + right_name;
        left_mask_name = path_prefix + left_mask_name;
        right_mask_name = path_prefix + right_mask_name;

        IplImage *left_image = cvLoadImage(left_name.c_str());
        IplImage *right_image = cvLoadImage(right_name.c_str());
        assert(left_image != NULL);
        assert(right_image != NULL);

        IplImage *left_mask = cvLoadImage(left_mask_name.c_str());
        IplImage *right_mask = cvLoadImage(right_mask_name.c_str());

        std::vector<feature> left_features, right_features;
        Common::extractFeatures(left_image, left_features);
        Common::extractFeatures(right_image, right_features);

        images.push_back(std::pair<IplImage*,IplImage*>(left_image,right_image));
        masks.push_back(std::pair<IplImage*,IplImage*>(left_mask,right_mask));

        std::vector<StereoFeature> stereo_features =
                StereoFeatureCorrelation::correlateFeatures(left_features, right_features, left_image->height);

        filterFeaturesOnScale(stereo_features, 4.0f);

        if(stereo_features.size() == 0){
            std::cout << left_name << " " << right_name << std::endl;
            std::cout << left_features.size() << " " << right_features.size() << std::endl;
            assert(left_features.size() != 0);
            assert(right_features.size() != 0);
            assert(false);
        }
        std::cout << "num stereo features: " << stereo_features.size() << std::endl;
        frame_features.push_back(stereo_features);
    }
}

MemoryEvaluator::~MemoryEvaluator(){
    assert(images.size() == masks.size());
    for(unsigned i = 0; i < images.size(); i++){
        cvReleaseImage(&images[i].first);
        cvReleaseImage(&images[i].second);

        cvReleaseImage(&masks[i].first);
        cvReleaseImage(&masks[i].second);
    }
}

void MemoryEvaluator::evaluate(ObjectSnapshotDB &snapshot_db,
                               unsigned &total_features,
                               unsigned &correct,
                               unsigned &incorrect){

    total_features = correct = incorrect = 0;
    for(unsigned i = 0; i < images.size(); i++){
/*
        for(unsigned j = 0; j < frame_features[i].size(); j++){
            StereoFeature sfeature = frame_features[i][j];
            if(isObjectFeature(sfeature)){
                if(isObjectPixel(masks[i].first, sfeature.feature_left.x, sfeature.feature_left.y) &&
                   isObjectPixel(masks[i].second, sfeature.feature_right.x, sfeature.feature_right.y)){
                    correct++;
                }
                else{
                    incorrect++;
                }
            }
        }
*/

        SceneMatchResult match_result = snapshot_db.matchScene(frame_features[i], true);

        std::set<unsigned>::iterator it;
        for(it = match_result.matched_scene_features.begin(); it != match_result.matched_scene_features.end(); ++it){
            StereoFeature feature = frame_features[i][*it];
            if(isObjectPixel(masks[i].first, feature.feature_left.x, feature.feature_left.y) &&
               isObjectPixel(masks[i].second, feature.feature_right.x, feature.feature_right.y)){
                correct++;
            }
            else{
                incorrect++;
            }
        }


        for(unsigned j = 0; j < frame_features[i].size(); j++){
            if(isObjectPixel(masks[i].first, frame_features[i][j].feature_left.x, frame_features[i][j].feature_left.y) &&
               isObjectPixel(masks[i].second, frame_features[i][j].feature_right.x, frame_features[i][j].feature_right.y)){
                total_features++;
            }
        }

    }
}

void MemoryEvaluator::loadBackgroundFeatures(std::string path, std::string root_file){
    std::ifstream in_file(root_file.c_str());

    unsigned num_frames = 0;
    in_file >> num_frames;
    std::cout << "num background frames: " << num_frames << std::endl;

    for(unsigned i = 0; i < num_frames; i++){
        std::string cur_filename;
        in_file >> cur_filename;
        cur_filename = path + cur_filename;

        IplImage *image = cvLoadImage(cur_filename.c_str());
        if(image == NULL){
            std::cout << cur_filename << std::endl;
            assert(false);
        }

        std::vector<feature> image_features;
        Common::extractFeatures(image, image_features);
        background_features.insert(background_features.end(), image_features.begin(), image_features.end());

        cvReleaseImage(&image);
    }

    std::cout << "background features: " << background_features.size() << std::endl;
    std::vector<float> dists;
    for(unsigned i = 0; i < 2000; i++){
        unsigned a = rand()%background_features.size();
        unsigned b = rand()%background_features.size();
        dists.push_back(sqrtf(descr_dist_sq(&background_features[a], &background_features[b])));
    }

    std::cout << "! " << Common::average(dists) << " " << Common::standardDeviation(dists) << std::endl;
}

void MemoryEvaluator::addObjectFeatures(std::vector<StereoFeature> &new_object_features){
    object_features.insert(object_features.end(), new_object_features.begin(), new_object_features.end());
}

bool MemoryEvaluator::isObjectFeature(StereoFeature sfeature){
    float closest_background_dist = 0.0f;
    for(unsigned i = 0; i < background_features.size(); i++){
        float cur_dist = sqrtf(descr_dist_sq(&(sfeature.feature_left), &(background_features[i]))) +
                         sqrtf(descr_dist_sq(&(sfeature.feature_right), &(background_features[i])));
        cur_dist *= 0.5f;
        if(i == 0 || cur_dist < closest_background_dist){
            closest_background_dist = cur_dist;
        }
    }

    float closest_object_dist = 0.0f;
    for(unsigned i = 0; i < object_features.size(); i++){
        float cur_dist = Common::featureDist(sfeature, object_features[i]);
        if(i == 0 || cur_dist < closest_object_dist){
            closest_object_dist = cur_dist;
        }
    }

    return closest_object_dist < 0.8f*closest_background_dist;
}

void MemoryEvaluator::evalManualSegmentation(std::string path_prefix, std::string root_file){
    ObjectSnapshotDB snapshot_db("");
    std::ifstream in_file(root_file.c_str());

    unsigned num_frames = 0;
    in_file >> num_frames;
    //std::cout << "num background frames: " << num_frames << std::endl;

    for(unsigned i = 0; i < num_frames; i++){
        std::string left_name, right_name;
        std::string left_mask_name, right_mask_name;

        in_file >> left_name;
        in_file >> right_name;

        in_file >> left_mask_name;
        in_file >> right_mask_name;

        left_name = path_prefix + left_name;
        right_name = path_prefix + right_name;
        left_mask_name = path_prefix + left_mask_name;
        right_mask_name = path_prefix + right_mask_name;

        IplImage *left_image = cvLoadImage(left_name.c_str());
        IplImage *right_image = cvLoadImage(right_name.c_str());
        assert(left_image != NULL);
        assert(right_image != NULL);

        IplImage *left_mask = cvLoadImage(left_mask_name.c_str());
        IplImage *right_mask = cvLoadImage(right_mask_name.c_str());

        std::vector<feature> left_features, right_features;
        Common::extractFeatures(left_image, left_features);
        Common::extractFeatures(right_image, right_features);

        std::vector<feature> filtered_left_features, filtered_right_features;
        for(unsigned j = 0; j < left_features.size(); j++){
            if(isObjectPixel(left_mask, left_features[j].x, left_features[j].y)){
                filtered_left_features.push_back(left_features[j]);
            }
        }
        for(unsigned j = 0; j < right_features.size(); j++){
            if(isObjectPixel(right_mask, right_features[j].x, right_features[j].y)){
                filtered_right_features.push_back(right_features[j]);
            }
        }

        std::vector<StereoFeature> stereo_features =
                StereoFeatureCorrelation::correlateFeatures(filtered_left_features, filtered_right_features, left_image->height);
        assert(stereo_features.size() > 0);
        snapshot_db.addSnapshot(stereo_features);

        cvReleaseImage(&left_image);
        cvReleaseImage(&right_image);
        cvReleaseImage(&left_mask);
        cvReleaseImage(&right_mask);
    }

    unsigned total, correct, incorrect;
    evaluate(snapshot_db, total, correct, incorrect);

    std::cout << "Manual segmentation result: " << std::endl;
    std::cout << (float)correct/(float)total << " " << (float)incorrect/(float)total << std::endl;
}

bool MemoryEvaluator::isObjectPixel(IplImage *mask, int xcoord, int ycoord){
    double r, g, b;
    Common::getPixel(mask, xcoord, ycoord, r, g, b);
    return !(r >= 254.0 && g >= 254.0 && b >= 254.0);
}

void MemoryEvaluator::filterFeaturesOnScale(std::vector<StereoFeature> &features, float max_scale){
    std::vector<StereoFeature> filtered_features;
    for(unsigned i = 0; i < features.size(); i++){
        if((features[i].feature_left.scl+features[i].feature_left.scl)/2.0f < max_scale){
            filtered_features.push_back(features[i]);
        }
    }
    features = filtered_features;
}
