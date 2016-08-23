/*
 * ReconstructionManager.cpp
 *
 *  Created on: 03/07/2009
 *      Author: osushkov
 */

#include "ReconstructionManager.h"
#include "BestFit.h"
#include "SuperQuadricBestFit.h"
#include "CentralisePoints.h"
#include "../Util/Common.h"
#include "../Util/Geometry.h"
#include "../Util/DebugVisualiser.h"
#include "../Util/Octree.hpp"
#include "../Util/Timer.h"
#include "../Visualisation/SceneRenderer.h"
#include "../Visualisation/SuperQuadricRenderObject.h"
#include "../Visualisation/LinkedPointsRenderObject.h"

#include <map>
#include <cassert>
#include <set>
#include <fstream>

#define MIN_MATCHING_FEATURES 3


ReconstructionManager::ReconstructionManager(std::string object_name) :
    object_name(object_name), cur_frame_id(0), cur_feature_id(0) {
    SceneRenderer::instance().initialise();

    point_cloud = new PointCloudRenderObject();
    arm_point_cloud = new PointCloudRenderObject();
}

ReconstructionManager::~ReconstructionManager(){
}


void ReconstructionManager::submitFeatures(const std::vector<StereoFeature> &object_features,
                                           const std::vector<StereoFeature> &arm_features,
                                           unsigned frame_id){
    ReconstructionFrame new_frame;

    std::vector<StereoFeature> filtered_object_features = object_features;
    for(unsigned i = 0; i < filtered_object_features.size(); i++){
        //object_features[i].position.print();

        ReconstructionFeature new_rfeature;

        new_rfeature.feature = filtered_object_features[i];
        new_rfeature.is_object = true;
        new_rfeature.is_linked = false;

        new_rfeature.id = cur_feature_id++;
        new_rfeature.transformed_position = new_rfeature.feature.position;

        new_frame.rfeatures.push_back(new_rfeature);
    }

    std::vector<StereoFeature> filtered_arm_features = arm_features;
    for(unsigned i = 0; i < filtered_arm_features.size(); i++){
        ReconstructionFeature new_rfeature;

        new_rfeature.feature = filtered_arm_features[i];
        new_rfeature.is_object = false;
        new_rfeature.is_linked = false;

        new_rfeature.id = cur_feature_id++;
        new_rfeature.transformed_position = new_rfeature.feature.position;

        new_frame.rfeatures.push_back(new_rfeature);
    }

    new_frame.snapshot = new ObjectSnapshot(object_name, cur_frame_id);
    new_frame.snapshot->addFeatures(filtered_object_features);
    new_frame.snapshot->addFeatures(filtered_arm_features);

    new_frame.is_linked = false;
    new_frame.frame_id = frame_id;


    std::pair<unsigned, ReconstructionFrame> new_entry(new_frame.frame_id, new_frame);
    reconstruction_frames.insert(new_entry);
}

void ReconstructionManager::submitFeatures(const std::vector<StereoFeature> &object_features,
                                           const std::vector<StereoFeature> &arm_features){
    submitFeatures(object_features, arm_features, cur_frame_id++);
}

std::vector<ModelFrame> ReconstructionManager::reconstruct(){

    linkFrames();
    for(unsigned i = 0; i < 2; i++){
        linkFeatures();
    }

    return generateModelFrames();
}

std::vector<Vector3D>
ReconstructionManager::getFeaturePositions(const std::vector<StereoFeature> &features){
    std::vector<Vector3D> result;
    for(unsigned i = 0; i < features.size(); i++){
        result.push_back(features[i].position);
    }
    return result;
}

unsigned ReconstructionManager::findNumberMatchingFeatures(const ReconstructionFrame &frame1,
                                                           const ReconstructionFrame &frame2){
    std::vector<StereoFeature> features = frame1.snapshot->getFeatures();
    std::vector<SnapshotMatchResult> matches;
    return frame2.snapshot->matchScene(features, matches, false);
}

Vector3D ReconstructionManager::applyTransform(const Transform &t, const Vector3D &vec){
    return t.mat*(vec - t.shift) + t.shift;
}

void ReconstructionManager::linkFrames(void){
    std::cout << "Linking Frames..." << std::endl;
    std::map<unsigned, ReconstructionFrame>::iterator it, it2;

    for(it = reconstruction_frames.begin(); it != reconstruction_frames.end(); ++it){
        ReconstructionFrame prev_frame, post_frame;

        for(unsigned i = 1; i < 5; i++){
            unsigned index;

            if(it->second.frame_id < i){
                index = reconstruction_frames.size()+it->second.frame_id-i;
            }
            else{
                index = it->second.frame_id-i;
            }

            std::vector<StereoFeature> features = it->second.snapshot->getFeatures();
            std::vector<SnapshotMatchResult> matches;
            unsigned num_matches =
                reconstruction_frames[index].snapshot->matchScene(features, matches, false);

            it->second.result_map[reconstruction_frames[index].frame_id] = matches;

            if(num_matches >= MIN_MATCHING_FEATURES){
                std::pair<unsigned,unsigned> new_frame(reconstruction_frames[index].frame_id, num_matches);
                it->second.linked_frames.push_back(new_frame);
            }
        }

        for(unsigned i = 1; i < 5; i++){
            unsigned index;

            if(it->second.frame_id >= reconstruction_frames.size()-i){
                index = it->second.frame_id+i-reconstruction_frames.size();
            }
            else{
                index = it->second.frame_id+i;
            }

            std::vector<StereoFeature> features = it->second.snapshot->getFeatures();
            std::vector<SnapshotMatchResult> matches;
            unsigned num_matches =
                reconstruction_frames[index].snapshot->matchScene(features, matches, false);

            it->second.result_map[reconstruction_frames[index].frame_id] = matches;

            if(num_matches >= MIN_MATCHING_FEATURES){
                std::pair<unsigned,unsigned> new_frame(reconstruction_frames[index].frame_id, num_matches);
                it->second.linked_frames.push_back(new_frame);
            }
        }


        std::cout << it->second.linked_frames.size() << std::endl;

    }
}

void ReconstructionManager::linkFeatures(void){
    std::cout << "Linking Features..." << std::endl;

    std::map<unsigned, ReconstructionFrame>::iterator it;
    std::map<unsigned, ReconstructionFrame>::reverse_iterator rit;

    std::set<unsigned> done_frames;
    for(it = reconstruction_frames.begin(); it != reconstruction_frames.end(); ++it){
        if(it->first == 0){
            it->second.is_linked = true;
        }

        if(done_frames.find(it->second.frame_id) == done_frames.end()){
            reconstructFrame(it->second);
            done_frames.insert(it->second.frame_id);
        }
    }

    std::cout << "second iteration" << std::endl;

    done_frames.clear();
    for(rit = reconstruction_frames.rbegin(); rit != reconstruction_frames.rend(); ++rit){
        if(done_frames.find(rit->second.frame_id) == done_frames.end()){
            reconstructFrame(rit->second);
            done_frames.insert(rit->second.frame_id);
        }
    }
}

void ReconstructionManager::reconstructFrame(ReconstructionFrame &frame) {
    if(frame.rfeatures.size() == 0){ return; }
    //if(frame.is_linked){ return; }

    std::vector<BestFitFeaturePair> all_pos_pairs;
    Vector3D shift(0.0f, 0.0f, 0.0f);
    unsigned num = 0;

    for (unsigned i = 0; i < frame.linked_frames.size(); i++) {
        if (!reconstruction_frames[frame.linked_frames[i].first].is_linked) {
            continue;
        }

        std::vector<unsigned> linked_frame_features;
        std::vector<BestFitFeaturePair> pos_pairs =
            linkFrameFeatures(frame, reconstruction_frames[frame.linked_frames[i].first],
                              linked_frame_features);

        if(pos_pairs.size() < 3){ continue; }

        for(unsigned j = 0; j < pos_pairs.size(); j++){
            all_pos_pairs.push_back(pos_pairs[j]);
        }

        for(unsigned j = 0; j < linked_frame_features.size(); j++){
            frame.rfeatures[linked_frame_features[j]].is_linked = true;
        }

        Vector3D rot_centre = findRotationCentre(pos_pairs);
        shift = shift + rot_centre;
        num++;
    }

    if(num == 0){ return; }

    Transform cur_transform;
    bool use_prev = false;//frame.frame_id != 0;
    best_fit.setHintShift(1.0f/num * shift);

    float error = best_fit.calculateBestFit(all_pos_pairs, cur_transform, use_prev);
    std::cout << "error: " << error << std::endl;
    if(error > 0.4f){
        return;
    }

    frame.transform = cur_transform;
    frame.is_linked = true;
    applyFrameTransform(frame);
}

std::vector<BestFitFeaturePair>
ReconstructionManager::linkFrameFeatures(ReconstructionFrame &src,
                                         ReconstructionFrame &dst,
                                         std::vector<unsigned> &linked_src_features){
    assert(dst.is_linked);

    std::vector<StereoFeature> src_features = src.snapshot->getFeatures();
    std::vector<StereoFeature> dst_features = dst.snapshot->getFeatures();

    assert(src_features.size() == src.rfeatures.size());
    assert(dst_features.size() == dst.rfeatures.size());

    std::vector<SnapshotMatchResult> indexes = src.result_map[dst.frame_id];
    std::vector<BestFitFeaturePair> pos_pairs;

    linked_src_features.clear();
    for(unsigned i = 0; i < indexes.size(); i++){
        if(src.rfeatures[indexes[i].index0].is_object != dst.rfeatures[indexes[i].index1].is_object){
            continue;
        }

        BestFitFeaturePair bffp;

        linked_src_features.push_back(indexes[i].index0);
        bffp.fpos0 = src.rfeatures[indexes[i].index0].transformed_position;
        bffp.fpos1 = dst.rfeatures[indexes[i].index1].transformed_position;
        bffp.pmatch = indexes[i].pmatch;
        pos_pairs.push_back(bffp);
    }

    return pos_pairs;
}

void ReconstructionManager::applyFrameTransform(ReconstructionFrame &frame){
    for(unsigned i = 0; i < frame.rfeatures.size(); i++){
        frame.rfeatures[i].transformed_position =
            applyTransform(frame.transform, frame.rfeatures[i].transformed_position);
    }
}

std::vector<ModelFrame> ReconstructionManager::generateModelFrames(void){
    std::vector<ModelFrame> result;
    std::vector<StereoFeature> object_features, arm_features;
    std::vector<Vector3D> arm_feature_positions;

    std::map<unsigned, ReconstructionFrame>::iterator it;
    for(it = reconstruction_frames.begin(); it != reconstruction_frames.end(); ++it){
       if(!it->second.is_linked){
           continue;
       }

       for(unsigned i = 0; i < it->second.rfeatures.size(); i++){
           if(!it->second.rfeatures[i].is_linked){ continue; }

           if(it->second.rfeatures[i].is_object){
               object_features.push_back( it->second.rfeatures[i].feature);
           }
           else{
               arm_features.push_back(it->second.rfeatures[i].feature);
               arm_feature_positions.push_back(it->second.rfeatures[i].feature.position);
           }
       }
    }

    const float max_arm_line_length = 2.0f;
    (void)max_arm_line_length;

    std::vector<std::pair<PointCloudPoint,PointCloudPoint> > point_pairs;

    /*
    std::vector< std::pair<Vector3D,Vector3D> > arm_lines;
    Octree<Vector3D> *octree = new Octree<Vector3D>(0.5f*(min_arm + max_arm),
                                                    max_arm.x-min_arm.x,
                                                    max_arm.y-min_arm.y,
                                                    max_arm.z-min_arm.z,
                                                    128, 8);
                                                    */

    Octree<StereoFeature> *arm_features_octree = buildOctree(arm_features);
    Octree<StereoFeature> *object_features_octree = buildOctree(object_features);


    /*
    for(unsigned i = 0; i < arm_feature_positions.size(); i++){
        octree->insertElement(arm_feature_positions[i], arm_feature_positions[i]);
    }

    for(unsigned i = 0; i < arm_feature_positions.size(); i++){
        std::vector<Vector3D> near_features =
            octree->getElementsInRegion(arm_feature_positions[i], max_arm_line_length);

        for(unsigned j = 0; j < near_features.size(); j++){
            if((arm_feature_positions[i]-near_features[j]).length() < 0.01f){ continue; }
            std::pair<Vector3D,Vector3D> new_pair(arm_feature_positions[i],near_features[j]);

            PointCloudPoint p0, p1;
            p0.pos = arm_feature_positions[i];
            p1.pos = near_features[j];

            std::pair<PointCloudPoint,PointCloudPoint> new_p(p0, p1);
            point_pairs.push_back(new_p);

            arm_lines.push_back(new_pair);
        }
    }
    */
/*
    for(unsigned i = 0; i < arm_feature_positions.size(); i++){
        for(unsigned j = i+1; j < arm_feature_positions.size(); j++){
            float d2 = (arm_feature_positions[i]-arm_feature_positions[j]).length2();
            if(d2 < max_arm_line_length2 && d2 > 0.01f){
                std::pair<Vector3D,Vector3D> new_pair(arm_feature_positions[i],arm_feature_positions[j]);

                PointCloudPoint p0, p1;
                p0.pos = arm_feature_positions[i];
                p1.pos = arm_feature_positions[j];

                std::pair<PointCloudPoint,PointCloudPoint> new_p(p0, p1);
                point_pairs.push_back(new_p);

                arm_lines.push_back(new_pair);
            }
        }
    }
*/
/*
    LinkedPointsRenderObject *linked_points = new LinkedPointsRenderObject();
    linked_points->addPoints(point_pairs);
    SceneRenderer::instance().addObject(linked_points);
*/


    std::cout << "total frames: " << reconstruction_frames.size() << std::endl;
    for(it = reconstruction_frames.begin(); it != reconstruction_frames.end(); ++it){
        if(!it->second.is_linked){
            continue;
        }
        std::cout << it->second.frame_id << std::endl;
        Util::Timer timer;
        timer.start();

        ModelFrame new_frame;
        for(unsigned i = 0; i < it->second.rfeatures.size(); i++){
            if(!it->second.rfeatures[i].is_linked || !it->second.rfeatures[i].is_object){
                continue;
            }

            /*if(!isObjectFeatureArmLineTest(it->second.rfeatures[i].feature, arm_lines)){
                continue;
            }*/

            if(!isObjectFeatureByWeight(it->second.rfeatures[i].feature,
                                        object_features_octree, arm_features_octree)){
                continue;
            }
/*
            if(!isObjectFeatureByWeight(it->second.rfeatures[i].feature,
                                        object_features, arm_features)){
                continue;
            }
*/
            StereoFeature new_feature = it->second.rfeatures[i].feature;
            new_feature.position = it->second.rfeatures[i].transformed_position;

            new_frame.features.push_back(new_feature);
        }

        new_frame.view_direction = it->second.transform.mat * Vector3D(0.0f, 0.0f, -1.0f);

        timer.stop();
        std::cout << timer.getNumElapsedSeconds() << std::endl;

        result.push_back(new_frame);
    }

    //delete octree;
    delete object_features_octree;
    delete arm_features_octree;

    return result;
}

Octree<StereoFeature>*
ReconstructionManager::buildOctree(const std::vector<StereoFeature> &features){
    Vector3D min(9999.0f, 9999.0f, 9999.0f), max(-9999.0f, -9999.0f, -9999.0f);

    for(unsigned i = 0; i < features.size(); i++){
        if(features[i].position.x < min.x){
            min.x = features[i].position.x;
        }
        if(features[i].position.y < min.y){
            min.y = features[i].position.y;
        }
        if(features[i].position.z < min.z){
            min.z = features[i].position.z;
        }
        if(features[i].position.x > max.x){
            max.x = features[i].position.x;
        }
        if(features[i].position.y > max.y){
            max.y = features[i].position.y;
        }
        if(features[i].position.z > max.z){
            max.z = features[i].position.z;
        }
    }

    Octree<StereoFeature> *octree = new Octree<StereoFeature>(0.5f*(min + max),
                                                              (max.x-min.x)*1.5f,
                                                              (max.y-min.y)*1.5f,
                                                              (max.z-min.z)*1.5f,
                                                              32, 16);
    for(unsigned i = 0; i < features.size(); i++){
        octree->insertElement(features[i].position, features[i]);
    }

    return octree;
}

bool ReconstructionManager::isObjectFeatureByWeight(const StereoFeature &f,
                                                    Octree<StereoFeature> *object_features_octree,
                                                    Octree<StereoFeature> *arm_features_octree){
    float object_weight = 0.0f;
    float arm_weight = 0.0f;

    std::vector<StereoFeature> near_object_features;
    object_features_octree->getElementsInRegion(f.position, 3.0f, near_object_features);

    std::vector<StereoFeature> near_arm_features;
    arm_features_octree->getElementsInRegion(f.position, 3.0f, near_arm_features);

    if(near_object_features.size() > 5.0f*near_arm_features.size()){
        return true;
    }

    if(near_arm_features.size() > 5.0f*near_object_features.size()){
        return false;
    }


    for(unsigned j = 0; j < near_object_features.size(); j++){
        float d = (f.position-near_object_features[j].position).length();
        object_weight += Common::normalDistribution(0.0f, 1.0f, d);
    }

    for(unsigned j = 0; j < near_arm_features.size(); j++){
        float d = (f.position-near_arm_features[j].position).length();
        arm_weight += Common::normalDistribution(0.0f, 3.0f, d);
    }

    if(object_weight > arm_weight){
        return true;
    }
    else{
        return false;
    }
}

bool ReconstructionManager::isObjectFeatureByWeight(const StereoFeature &f,
                                                    const std::vector<StereoFeature> &object_features,
                                                    const std::vector<StereoFeature> &arm_features){
    float object_weight = 0.0f;
    float arm_weight = 0.0f;

    std::vector<StereoFeature> near_object_features;
    std::vector<StereoFeature> near_arm_features;

    for(unsigned i = 0; i < object_features.size(); i++){
        if((object_features[i].position - f.position).length() < 3){
            near_object_features.push_back(object_features[i]);
        }
    }

    for(unsigned i = 0; i < arm_features.size(); i++){
        if((arm_features[i].position - f.position).length() < 3){
            near_arm_features.push_back(arm_features[i]);
        }
    }

    if(near_object_features.size() > 4.0f*near_arm_features.size()){
        return true;
    }

    if(near_arm_features.size() > 4.0f*near_object_features.size()){
        return false;
    }


    for(unsigned j = 0; j < near_object_features.size(); j++){
        float d = (f.position-near_object_features[j].position).length();
        object_weight += Common::normalDistribution(0.0f, 1.0f, d);
    }

    for(unsigned j = 0; j < near_arm_features.size(); j++){
        float d = (f.position-near_arm_features[j].position).length();
        arm_weight += Common::normalDistribution(0.0f, 2.0f, d);
    }

    if(object_weight > arm_weight){
        return true;
    }
    else{
        return false;
    }


}

bool ReconstructionManager::isObjectFeatureArmLineTest
(const StereoFeature &f, const std::vector<std::pair<Vector3D,Vector3D> > &arm_lines){
    for(unsigned i = 0; i < arm_lines.size(); i++){
        float t = Geometry::linePointDistanceT(arm_lines[i].first, arm_lines[i].second, f.position);
        float dist = Geometry::linePointDistance(arm_lines[i].first, arm_lines[i].second, f.position);
        if(t >= 0.0f && t <= 1.0f && dist < 1.0f){
            return false;
        }
    }

    return true;
}

Vector3D ReconstructionManager::findRotationCentre(std::vector<BestFitFeaturePair> pos_pairs){
    Vector3D result(0.0f, 0.0f, 0.0f);

    unsigned num_checks = 30;
    unsigned num_valid = 0;
    for(unsigned i = 0; i < num_checks; i++){
        unsigned index0, index1, index2;

        index0 = rand()%pos_pairs.size();
        while((index1 = rand()%pos_pairs.size()) == index0);
        while((index2 = rand()%pos_pairs.size()) == index0 || index2 == index1);

        std::vector<Vector3D> src_triangle, dst_triangle;
        src_triangle.push_back(pos_pairs[index0].fpos0);
        src_triangle.push_back(pos_pairs[index1].fpos0);
        src_triangle.push_back(pos_pairs[index2].fpos0);

        dst_triangle.push_back(pos_pairs[index0].fpos1);
        dst_triangle.push_back(pos_pairs[index1].fpos1);
        dst_triangle.push_back(pos_pairs[index2].fpos1);

        if(Geometry::triangleArea(src_triangle) < 0.1f ||
           Geometry::triangleArea(dst_triangle) < 0.1f){
            continue;
        }

        std::vector<Vector3D> src_basis = Common::basisVectorsFromTriangle(src_triangle);
        std::vector<Vector3D> dst_basis = Common::basisVectorsFromTriangle(dst_triangle);

        Vector3D line0_start = src_triangle[0] + 50.0f*src_basis[0];
        Vector3D line0_end = src_triangle[0] - 50.0f*src_basis[0];

        Vector3D line1_start = dst_triangle[0] + 50.0f*dst_basis[0];
        Vector3D line1_end = dst_triangle[0] - 50.0f*dst_basis[0];

        float a, b;
        Vector3D p0, p1;
        if(Geometry::lineIntersect(line0_start, line0_end, line1_start, line1_end, p0, p1, a, b)){
            Vector3D intersect_point = 0.5f*(p0 + p1);
            result = result + intersect_point;

            num_valid++;
        }
    }

    if(num_valid == 0){
        return Vector3D(0.0f, 0.0f, 0.0f);
    }

    assert(num_valid > 0);
    result.scale(1.0f/num_valid);

    return result;
}
