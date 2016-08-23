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
#include "../Features/FeatureMemory/ObjectSnapshot.h"
#include "../Features/FeatureMemory/ObjectSnapshotDB.h"
#include "../Util/Common.h"
#include "../Util/Geometry.h"
#include "../Util/DebugVisualiser.h"
#include "../Util/Octree.hpp"
#include "../Util/Timer.h"
#include "../Visualisation/SceneRenderer.h"
#include "../Visualisation/SuperQuadricRenderObject.h"
#include "../Visualisation/LinkedPointsRenderObject.h"
#include "../Match.h"
#include "ICPMatch.h"

#include <map>
#include <cassert>
#include <set>
#include <fstream>

#define MIN_MATCHING_FEATURES 3


ReconstructionManager::ReconstructionManager(std::string object_name) :
    object_name(object_name) {
    SceneRenderer::instance().initialise();

    object_point_cloud = new PointCloudRenderObject(false, Vector3D(0.0f, 1.0f, 0.0f));
    arm_point_cloud = new PointCloudRenderObject(false, Vector3D(1.0f, 0.0f, 0.0f));
}

ReconstructionManager::ReconstructionManager(){

}

ReconstructionManager::~ReconstructionManager(){
}



std::vector<SnapshotFrame> ReconstructionManager::reconstruct(const std::vector<SnapshotFrame> &snapshot_frames){
    SceneRenderer::instance().addObject(object_point_cloud);
    SceneRenderer::instance().addObject(arm_point_cloud);

    genModelFrames(snapshot_frames);
    filterModelFrames();


    const unsigned num_iters = 2;
    for(unsigned iter = 0; iter <= num_iters; iter++){
        std::cout << "iter: " << iter << std::endl;
  
        if(iter == 1){ 
            frame_match_data.clear();
            centraliseModelFrames(); 
        }

        for(unsigned i = 0; i < model_frames.size(); i++){
            model_frames[i].behind = false;
        }

        object_point_cloud->clearPoints();
        arm_point_cloud->clearPoints();

        std::vector<Vector3D> all_object_points;

        int start, end, inc;
        if(iter%2 == 0){
            start = iter == 0 ? 1 : 0;
            end = model_frames.size();
            inc = 1;
        }
        else{
            start =  model_frames.size();
            end = -1;
            inc = -1;
        }

        for(int i = start; i != end; i += inc){
            int index;
            if(i%2 == 0){
                index = i/2;
            }
            else{
                index = -i/2 - 1;
            }

            if(index < 0){ index += model_frames.size(); }

            
            reconstructFrame(index, iter/(float)num_iters);
            model_frames[index].frame.to_camera.print();

            if(model_frames[index].is_valid){
                insertObjectPixels(all_object_points, model_frames[index]);
                all_object_points = ICPMatch::filterPoints(all_object_points, 0.2f);
                object_point_cloud->clearPoints();

                std::vector<PointCloudPoint> new_obj_points;
                for(unsigned j = 0; j < all_object_points.size(); j++){
                    PointCloudPoint new_point;
                    new_point.pos = all_object_points[j];
                    new_point.color = Vector3D(1.0f, 0.0f, 0.0f);
                    new_obj_points.push_back(new_point);
                }
                object_point_cloud->addPoints(new_obj_points);
            }
        }
 
    }

    //addFrameToPointClouds(model_frames[1]);
    std::vector<SnapshotFrame> result;
    for(unsigned i = 0; i < model_frames.size(); i++){
        if(model_frames[i].is_valid){
            result.push_back(model_frames[i].frame);
            result.back().to_camera.print();
        }
    }
    getchar();

    return result;
}

void ReconstructionManager::stitchTogether(std::vector<SnapshotFrame> &frames0, std::vector<SnapshotFrame> &frames1){
    cleanUpFrames(frames0);
    cleanUpFrames(frames1);

    std::vector<ObjectSnapshot> snapshots0;
    for(unsigned i = 0; i < frames0.size(); i++){
        ObjectSnapshot new_snapshot("temp", i);
        new_snapshot.addFeatures(frames0[i].object_features, std::vector<SurfacePoint>());
        snapshots0.push_back(new_snapshot);
    }

    std::cout << snapshots0.size() << " ! " << frames1.size() << std::endl;
    float best = 0.0f;
    Transform best_transform;
    MatchData best_match_data;

    for(unsigned j = 0; j < frames1.size(); j++){
        std::cout << j << std::endl;

        ObjectSnapshotDB db("temp");
        for(unsigned i = 0; i < snapshots0.size(); i++){
            db.addSnapshot(&(snapshots0[i]));
        }

        SceneMatchResult smr = db.matchScene(frames1[j].object_features);
        if(smr.match_score > best && smr.matched_snapshot != NULL){
            best = smr.match_score;
            best_transform = smr.approximate_transform;
            std::cout << "best: " << best << std::endl;

            best_match_data.src_features = smr.matched_snapshot->getFeatures();
            best_match_data.dst_features = frames1[j].object_features;
            best_match_data.feature_matches = smr.feature_matches;
            best_match_data.approx_transform = best_transform;

            if(best >= 24){
                break;
            }
        }
    }

    float err;
    Match match_refine;
    best_transform = match_refine.doMatchApprox(best_match_data, err);

    for(unsigned i = 0; i < frames0.size(); i++){
        applyTransformToSnapshotFrame(best_transform, frames0[i]);
    }

    for(unsigned i = 0; i < best_match_data.src_features.size(); i++){
        Vector3D p = best_match_data.src_features[i].position;
        p = best_transform.mat*(p - best_transform.secondary_shift) + best_transform.shift;
        best_match_data.src_features[i].position = p;
    }

    best_transform = match_refine.doMatch(best_match_data, err);

    for(unsigned i = 0; i < frames0.size(); i++){
        applyTransformToSnapshotFrame(best_transform, frames0[i]);
    }


    std::vector<Vector3D> frame0_points = getAllObjectPixels(frames0);
    std::vector<Vector3D> frame1_points = getAllObjectPixels(frames1);

    frame0_points = ICPMatch::filterPoints(frame0_points, 0.2f);
    frame1_points = ICPMatch::filterPoints(frame1_points, 0.2f);

    Transform icpt = ICPMatch::match(frame0_points, frame1_points, err);

    for(unsigned i = 0; i < frames0.size(); i++){
        applyTransformToSnapshotFrame(icpt, frames0[i]);
    }
/*
    for(unsigned iter = 0; iter < 1; iter++){
        for(unsigned i = 0; i < frames0.size(); i++){
            frame0_points = getAllObjectPixels(frames0, i);
            frame1_points = getAllObjectPixels(frames1);
            std::vector<Vector3D> dst_points = frame0_points;
            dst_points.insert(dst_points.end(), frame1_points.begin(), frame1_points.end());

            std::vector<Vector3D> src_points = getAllObjectPixels(frames0[i]);

            src_points = ICPMatch::filterPoints(src_points, 0.2f);
            dst_points = ICPMatch::filterPoints(dst_points, 0.2f);

            icpt = ICPMatch::match(src_points, dst_points, err);
            applyTransformToSnapshotFrame(icpt, frames0[i]);
        }

        for(unsigned i = 0; i < frames1.size(); i++){
            frame0_points = getAllObjectPixels(frames0);
            frame1_points = getAllObjectPixels(frames1, i);
            std::vector<Vector3D> dst_points = frame0_points;
            dst_points.insert(dst_points.end(), frame1_points.begin(), frame1_points.end());

            std::vector<Vector3D> src_points = getAllObjectPixels(frames1[i]);

            src_points = ICPMatch::filterPoints(src_points, 0.2f);
            dst_points = ICPMatch::filterPoints(dst_points, 0.2f);

            icpt = ICPMatch::match(src_points, dst_points, err);
            applyTransformToSnapshotFrame(icpt, frames1[i]);
        }
    }
    */
}

std::vector<Vector3D> ReconstructionManager::getAllObjectPixels(const std::vector<SnapshotFrame> &frames, int except){
    std::vector<Vector3D> result;
    for(int i = 0; i < frames.size(); i++){
        if(i != except){
            std::vector<Vector3D> frame_pixels = getAllObjectPixels(frames[i]);
            result.insert(result.end(), frame_pixels.begin(), frame_pixels.end());
        }
    }
    return result;
}

std::vector<Vector3D> ReconstructionManager::getAllObjectPixels(const SnapshotFrame &frame){
    std::vector<Vector3D> result;
    for(unsigned j = 0; j < frame.object_pixels.size(); j++){
        result.push_back(frame.object_pixels[j].pos);
    }
    return result;
}

void ReconstructionManager::cleanUpFrames(std::vector<SnapshotFrame> &frames){
    std::vector<Vector3D> all_pixels;
    std::vector<Vector3D> all_obj_pixels;
    for(unsigned i = 0; i < frames.size(); i++){
        for(unsigned j = 0; j < frames[i].object_pixels.size(); j++){
            all_pixels.push_back(frames[i].object_pixels[j].pos);
            all_obj_pixels.push_back(frames[i].object_pixels[j].pos);
        }
        for(unsigned j = 0; j < frames[i].arm_pixels.size(); j++){
            all_pixels.push_back(frames[i].arm_pixels[j].pos);
        }
    }

    Vector3D centre = Geometry::centreOfMass(all_pixels);
    float width = 0.0f, height = 0.0f, depth = 0.0f;

    for(unsigned i = 0; i < all_pixels.size(); i++){
        float xd = fabs(all_pixels[i].x - centre.x);
        float yd = fabs(all_pixels[i].y - centre.y);
        float zd = fabs(all_pixels[i].z - centre.z);

        if(xd > width){ width = xd; }
        if(yd > height){ height = yd; }
        if(zd > depth){ depth = zd; }
    }

    Octree<int> octree(centre, width*2.0f, height*2.0f, depth*2.0f, 32, 8);
    for(unsigned i = 0; i < frames.size(); i++){
        for(unsigned j = 0; j < frames[i].object_pixels.size(); j++){
            octree.insertElement(frames[i].object_pixels[j].pos, 0);
        }
        for(unsigned j = 0; j < frames[i].arm_pixels.size(); j++){
            octree.insertElement(frames[i].arm_pixels[j].pos, 1);
        }
    }

    Vector3D obj_cog = Geometry::centreOfMass(all_obj_pixels);
    const float max_dist = 20.0f;
    for(unsigned i = 0; i < frames.size(); i++){
        std::vector<SIFTFeature3D> filtered_obj_features;
        for(unsigned j = 0; j < frames[i].object_features.size(); j++){
            std::vector<int> near_pixels;
            octree.getElementsInRegion(frames[i].object_features[j].position, 0.5f, near_pixels);

            unsigned num_arm = 0, num_obj = 0;
            for(unsigned k = 0; k < near_pixels.size(); k++){
                if(near_pixels[k] == 0){ num_obj++; }
                else{ num_arm++; }
            }

            if(num_obj > num_arm && (obj_cog - frames[i].object_features[j].position).length() < max_dist){
                filtered_obj_features.push_back(frames[i].object_features[j]);
            }
        }
        frames[i].object_features = filtered_obj_features;

        std::vector<KinectCamera::DepthPixel> filtered_obj_pixels;
        for(unsigned j = 0; j < frames[i].object_pixels.size(); j++){
            std::vector<int> near_pixels;
            octree.getElementsInRegion(frames[i].object_pixels[j].pos, 0.5f, near_pixels);

            unsigned num_arm = 0, num_obj = 0;
            for(unsigned k = 0; k < near_pixels.size(); k++){
                if(near_pixels[k] == 0){ num_obj++; }
                else{ num_arm++; }
            }

            if(num_obj > num_arm && (obj_cog - frames[i].object_pixels[j].pos).length() < max_dist){
                filtered_obj_pixels.push_back(frames[i].object_pixels[j]);
            }
        }
        frames[i].object_pixels = filtered_obj_pixels;
    }
}

void ReconstructionManager::reconstructFrame(unsigned index, float iter_fraction){
    std::map<unsigned, ModelMatchData>::iterator it = frame_match_data.find(index);

    ModelMatchData match_data;
    if(it == frame_match_data.end()){ match_data = genModelMatchData(index); }
    else{ match_data = it->second; }

    if(match_data.best_match_score < 0.001f){
        model_frames[index].is_valid = false;
        return;
    }

    Match match_refine;
    MatchData match_refine_data;

    if(iter_fraction < 0.999f || true){
        match_refine_data.src_features = model_frames[index].snapshot->getFeatures();

        std::cout << "m: " << match_data.best_match_score << std::endl;
        for(unsigned j = 0; j < match_data.all_match_scores.size(); j++){
            if(true || match_data.all_match_scores[j] >= 0.75f*match_data.best_match_score){
                unsigned offset = match_refine_data.dst_features.size();

                std::vector<SIFTFeature3D> mf = model_frames[match_data.all_frame_features_index[j]].snapshot->getFeatures();
                match_refine_data.dst_features.insert(match_refine_data.dst_features.end(), mf.begin(), mf.end());               

                float weight = model_frames[match_data.all_frame_features_index[j]].behind ? 1.0f : (0.1f + 0.9f*iter_fraction);
                for(unsigned k = 0; k < match_data.all_matches[j].size(); k++){
                    SnapshotMatchResult m = match_data.all_matches[j][k];
                    m.pmatch = weight;
                    /*
                    if(m.scene_index >= model_frames[match_data.all_frame_features_index[j]].frame.arm_features.size()){
                        m.pmatch *= 10.0f;
                    }
                    */
                    m.scene_index += offset;
                    match_refine_data.feature_matches.push_back(m);
                }
            }
        }

        match_refine_data.approx_transform = match_data.best_transform;
        float err;
        if(iter_fraction < 0.001f){
            match_data.best_transform = match_refine.doMatchApprox(match_refine_data, err);
        }
        else{
            match_data.best_transform = match_refine.doMatch(match_refine_data, err);
        }

        model_frames[index].is_valid = true;

        applyTransformToFrame(match_data.best_transform, model_frames[index]);
    }

    std::vector<Vector3D> cur_rpixels = getReconstructionPixels(model_frames[index], iter_fraction);
    std::vector<Vector3D> model_pixels = getAllReconstructionPixels(index, iter_fraction);

    if(model_pixels.size() > 0){
        std::vector<Vector3D> cur_model_points = ICPMatch::filterPoints(cur_rpixels, 0.2f);
        std::vector<Vector3D> all_model_points = ICPMatch::filterPoints(model_pixels, 0.2f);

        float err;
        match_data.best_transform = ICPMatch::match(cur_model_points, all_model_points, err);
        if(err > 0.1 && iter_fraction > 0.9999f && false){
            model_frames[index].is_valid = false;
        }

        applyTransformToFrame(match_data.best_transform, model_frames[index]);
    }

    model_frames[index].glued = true;
    model_frames[index].behind = true;

    match_data.best_transform = Transform::identity();
    frame_match_data[index] = match_data;
}

void ReconstructionManager::genModelFrames(const std::vector<SnapshotFrame> &snapshot_frames){
    for(unsigned i = 0; i < snapshot_frames.size(); i++){
        ModelFrame new_model_frame;
        new_model_frame.frame = snapshot_frames[i];

        new_model_frame.snapshot = new ObjectSnapshot(object_name, i);
        new_model_frame.snapshot->addFeatures(snapshot_frames[i].arm_features, std::vector<SurfacePoint>());
        new_model_frame.snapshot->addFeatures(snapshot_frames[i].object_features, std::vector<SurfacePoint>());

        new_model_frame.view_direction = snapshot_frames[i].to_camera;
        new_model_frame.glued = false;
        new_model_frame.is_valid = true;
        new_model_frame.behind = false;
        model_frames.push_back(new_model_frame);
    }
    model_frames[0].glued = true;
    model_frames[0].behind = true;
}

ModelMatchData ReconstructionManager::genModelMatchData(unsigned mode_frame_index){
    ModelMatchData new_model_match_data;
    new_model_match_data.best_match_score = 0.0f;

    for(int offset = -10; offset < 10; offset++){
        if(offset == 0){ continue; }

        int index = mode_frame_index+offset;
        if(index < 0){ index += model_frames.size(); }
        if(index >= model_frames.size()){ index -= model_frames.size(); }
        if(!model_frames[index].glued || !model_frames[index].is_valid){ continue; }

        Transform cur_transform;
        std::vector<SnapshotMatchResult> matches;
        std::vector<SIFTFeature3D> frame_features = model_frames[index].snapshot->getFeatures(); 
        float r = model_frames[mode_frame_index].snapshot->matchScene(frame_features, matches, cur_transform);

        if(r > new_model_match_data.best_match_score){
            new_model_match_data.best_match_score = r;
            new_model_match_data.best_transform = cur_transform;
        }

        new_model_match_data.all_match_scores.push_back(r);
        new_model_match_data.all_frame_features_index.push_back(index);
        new_model_match_data.all_matches.push_back(matches);
    }

    return new_model_match_data;
}

void ReconstructionManager::addFrameToPointClouds(const ModelFrame &frame){
    std::vector<PointCloudPoint> new_arm_points;
    for(unsigned i = 0; i < frame.frame.arm_pixels.size(); i++){
        PointCloudPoint new_point;
        new_point.pos = frame.frame.arm_pixels[i].pos;
        new_point.color = Vector3D(1.0f, 0.0f, 0.0f);
        new_arm_points.push_back(new_point);
    }
    arm_point_cloud->addPoints(new_arm_points);

    std::vector<PointCloudPoint> new_object_points;
    for(unsigned i = 0; i < frame.frame.object_pixels.size(); i++){
        PointCloudPoint new_point;
        new_point.pos = frame.frame.object_pixels[i].pos;
        new_point.color = Vector3D(0.0f, 1.0f, 0.0f);
        new_arm_points.push_back(new_point);
    }
    object_point_cloud->addPoints(new_arm_points);
}

void ReconstructionManager::applyTransformToFrame(Transform t, ModelFrame &frame){
    frame.view_direction = t.mat*frame.view_direction;
    frame.frame.to_camera = t.mat*frame.frame.to_camera;

    for(unsigned i = 0; i < frame.frame.arm_pixels.size(); i++){
        frame.frame.arm_pixels[i].pos = t.mat*(frame.frame.arm_pixels[i].pos - t.secondary_shift) + t.shift;
    }

    for(unsigned i = 0; i < frame.frame.object_pixels.size(); i++){
        frame.frame.object_pixels[i].pos = t.mat*(frame.frame.object_pixels[i].pos - t.secondary_shift) + t.shift;
    }

    for(unsigned i = 0; i < frame.frame.arm_features.size(); i++){
        frame.frame.arm_features[i].position = t.mat*(frame.frame.arm_features[i].position - t.secondary_shift) + t.shift;
    }

    for(unsigned i = 0; i < frame.frame.object_features.size(); i++){
        frame.frame.object_features[i].position = t.mat*(frame.frame.object_features[i].position - t.secondary_shift) + t.shift;
    }

    std::vector<SIFTFeature3D> snapshot_features = frame.snapshot->getFeatures();
    std::vector<SurfacePoint> surface_points = frame.snapshot->getSurfacePoints();

    for(unsigned i = 0; i < snapshot_features.size(); i++){
        snapshot_features[i].position = t.mat*(snapshot_features[i].position - t.secondary_shift) + t.shift;
    }
    
    for(unsigned i = 0; i < surface_points.size(); i++){
        surface_points[i].pos = t.mat*(surface_points[i].pos - t.secondary_shift) + t.shift;
    }

    frame.snapshot->clearFeatures();
    frame.snapshot->clearSurfacePoints();
    frame.snapshot->addFeatures(snapshot_features, surface_points);
}

void ReconstructionManager::applyTransformToSnapshotFrame(Transform t, SnapshotFrame &frame){
    frame.to_camera = t.mat*frame.to_camera;

    for(unsigned j = 0; j < frame.object_features.size(); j++){
        frame.object_features[j].position = t.mat*(frame.object_features[j].position - t.secondary_shift) + t.shift;
    }

    for(unsigned j = 0; j < frame.object_pixels.size(); j++){
        frame.object_pixels[j].pos = t.mat*(frame.object_pixels[j].pos - t.secondary_shift) + t.shift;
    }
}

void ReconstructionManager::insertObjectPixels(std::vector<Vector3D> &dst, const ModelFrame &frame){
    /*
    for(unsigned i = 0; i < frame.frame.arm_pixels.size(); i++){
        dst.push_back(frame.frame.arm_pixels[i].pos);
    }
    */

    for(unsigned i = 0; i < frame.frame.object_pixels.size(); i++){
        dst.push_back(frame.frame.object_pixels[i].pos);
    }
}

std::vector<Vector3D> ReconstructionManager::getAllReconstructionPixels(unsigned except_index, float iter_frac){
    std::vector<Vector3D> result;

    for(unsigned i = 0; i < model_frames.size(); i++){
        if(model_frames[i].glued && model_frames[i].is_valid && i != except_index && model_frames[i].behind){
            if(iter_frac < 0.999f || true){
                for(unsigned j = 0; j < model_frames[i].frame.arm_pixels.size(); j++){
                    result.push_back(model_frames[i].frame.arm_pixels[j].pos);
                }
            }

            for(unsigned j = 0; j < model_frames[i].frame.object_pixels.size(); j++){
                result.push_back(model_frames[i].frame.object_pixels[j].pos);
            }
        }
    }

    return result;
}

std::vector<Vector3D> ReconstructionManager::getReconstructionPixels(const ModelFrame &frame, float iter_frac){
    std::vector<Vector3D> result;

    if(iter_frac < 0.999f || true){
        for(unsigned j = 0; j < frame.frame.arm_pixels.size(); j++){
            result.push_back(frame.frame.arm_pixels[j].pos);
        }
    }

    for(unsigned j = 0; j < frame.frame.object_pixels.size(); j++){
        result.push_back(frame.frame.object_pixels[j].pos);
    }
    
    return result;
}

void ReconstructionManager::centraliseModelFrames(void){
    std::vector<Vector3D> all_points;
    for(unsigned i = 0; i < model_frames.size(); i++){
        std::vector<Vector3D> model_points = getReconstructionPixels(model_frames[i], 0);
        all_points.insert(all_points.end(), model_points.begin(), model_points.end());
    }
    Vector3D cog = Geometry::centreOfMass(all_points);
    Transform t;
    t.mat.identity();
    t.shift = -1.0f * cog;
    t.secondary_shift = Vector3D(0.0f, 0.0f, 0.0f);

    for(unsigned i = 0; i < model_frames.size(); i++){
        applyTransformToFrame(t, model_frames[i]);
    }
}

void ReconstructionManager::filterModelFrames(void){
    const float radius = 20.0f;

    for(unsigned i = 0; i < model_frames.size(); i++){
        std::vector<Vector3D> object_points;
        for(unsigned j = 0; j < model_frames[i].frame.object_pixels.size(); j++){
            object_points.push_back(model_frames[i].frame.object_pixels[j].pos);
        }
        Vector3D cog = Geometry::centreOfMass(object_points);

        ModelFrame rframe;
        rframe.frame.to_camera = model_frames[i].frame.to_camera;
        rframe.frame.object_features = model_frames[i].frame.object_features;
        rframe.frame.object_pixels = model_frames[i].frame.object_pixels;

        for(unsigned j = 0; j < model_frames[i].frame.arm_pixels.size(); j++){
            if((model_frames[i].frame.arm_pixels[j].pos - cog).length() < radius){
                rframe.frame.arm_pixels.push_back(model_frames[i].frame.arm_pixels[j]);
            }
        }

        for(unsigned j = 0; j < model_frames[i].frame.arm_features.size(); j++){
            if((model_frames[i].frame.arm_features[j].position - cog).length() < radius){
                rframe.frame.arm_features.push_back(model_frames[i].frame.arm_features[j]);
            }
        }

        std::vector<SIFTFeature3D> snapshot_features = model_frames[i].snapshot->getFeatures();
        std::vector<SurfacePoint> surface_points = model_frames[i].snapshot->getSurfacePoints();

        std::vector<SIFTFeature3D> filtered_snapshot_features;
        std::vector<SurfacePoint> filtered_surface_points;

        for(unsigned j = 0; j < snapshot_features.size(); j++){
            if((snapshot_features[j].position - cog).length() < radius){
                filtered_snapshot_features.push_back(snapshot_features[j]);
            }
        }

        for(unsigned j = 0; j < surface_points.size(); j++){
            if((surface_points[j].pos - cog).length() < radius){
                filtered_surface_points.push_back(surface_points[j]);
            }
        }

        rframe.snapshot = model_frames[i].snapshot;
        rframe.snapshot->clearFeatures();
        rframe.snapshot->clearSurfacePoints();
        rframe.snapshot->addFeatures(filtered_snapshot_features, filtered_surface_points);

        rframe.view_direction = model_frames[i].view_direction;
        rframe.glued = model_frames[i].glued;
        rframe.is_valid = model_frames[i].is_valid;
        rframe.behind = model_frames[i].behind;

        model_frames[i] = rframe;
    } 
}