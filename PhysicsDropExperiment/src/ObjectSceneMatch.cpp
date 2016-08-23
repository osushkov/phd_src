/*
 * ObjectSceneMatch.cpp
 *
 *  Created on: 10/12/2009
 *      Author: osushkov
 */

#include "ObjectSceneMatch.h"
#include "Util/Timer.h"
#include "Util/Matrix.h"

#define _USE_MATH_DEFINES
#include <math.h>

ObjectSceneMatch::ObjectSceneMatch(std::string object_name,
                                   const std::vector<ModelFrame> &object_frames) :
                                       object_name(object_name),
                                       frames_since_last_seen(-1),
                                       have_last_result(false) {

    for(unsigned i = 0; i < object_frames.size(); i++){
        ObjectSnapshot *new_snapshot = new ObjectSnapshot(object_name, i);
        new_snapshot->addFeatures(object_frames[i].features);

        std::pair<Vector3D,ObjectSnapshot*> new_entry(object_frames[i].view_direction, new_snapshot);
        snapshot_ag.push_back(new_entry);
    }
}


ObjectSceneMatch::~ObjectSceneMatch() {

}

bool ObjectSceneMatch::sceneMatch(const std::vector<StereoFeature> &scene_features,
                                  Transform &object_transform,
                                  unsigned &num_features_matched) {

    num_features_matched = 0;
    pose_filter.performMotionUpdate();

    float best_result = 0.0f;
    unsigned best_index = 0;
    std::vector<SnapshotMatchResult> best_matches;
    Transform best_approx_transform;

    Util::Timer timer;
    timer.start();

    float max_angle_from_last = 180.0f*(float)M_PI/180.0f;
    if(frames_since_last_seen < 20.0f){
        max_angle_from_last = (frames_since_last_seen+1)*10.0f*(float)M_PI/180.0f;
    }

    unsigned num_checks = 0;

    for(unsigned i = 0; i < snapshot_ag.size(); i++){
        float angle = acosf(last_normal_vector.dotProduct(snapshot_ag[i].first));
        if(have_last_result && angle > max_angle_from_last){
            //continue;
        }

        std::vector<SnapshotMatchResult> matches;
        num_checks++;

        Transform approx_transform;
        float r = snapshot_ag[i].second->matchScene(scene_features, matches, approx_transform, false);
        if(r > best_result){
            best_result = r;
            best_index = i;
            best_matches = matches;
            best_approx_transform = approx_transform;
        }

        if(best_result > 15){
            //break;
        }

        //std::cout << i << "/" << snapshot_ag.size() << std::endl;
    }

    timer.stop();

    std::cout << "nc: " << num_checks << " " << best_result << " "
              << timer.getNumElapsedSeconds()*1000.0f << std::endl;


    if(best_result >= 5.0f){
        std::vector<BestFitFeaturePair> pos_pairs;
        std::vector<StereoFeature> snapshot_features = snapshot_ag[best_index].second->getFeatures();
        Transform transform;

        for (unsigned i = 0; i < best_matches.size(); i++) {
            BestFitFeaturePair bffp;
            bffp.fpos0 = scene_features[best_matches[i].index0].position;
            bffp.fpos1 = snapshot_features[best_matches[i].index1].position;
            bffp.pmatch = best_matches[i].pmatch;
            pos_pairs.push_back(bffp);
        }

        //Transform cam_transform = Common::cameraToWorldSpaceTransform(0.0f, -65.0f, 0.0);
        //transform.shift = Common::cameraPointToWorldSpace(transform.shift, 0.0f, -65.0f, 0.0);
        //transform.shift = cam_transform.mat*transform.shift + cam_transform.shift;
        //transform.mat = cam_transform.mat*transform.mat;



        best_fit.setHint(best_approx_transform);
        Util::Timer fit_timer;
        fit_timer.start();
        float error = best_fit.calculateBestFit(pos_pairs, transform, false);
        fit_timer.stop();
        std::cout << "error: " << error << " " << pos_pairs.size() << std::endl;
        std::cout << "timer: " << fit_timer.getNumElapsedSeconds()*1000.0f << "ms" << std::endl;

        if(error > 0.1f){
            best_fit.reset();
            frames_since_last_seen++;

            return false;
        }

        Matrix<float,7,1> measurement;
        Matrix<float,7,1> measurement_error;

        measurement(0,0) = transform.shift.x;
        measurement(1,0) = transform.shift.y;
        measurement(2,0) = transform.shift.z;

        measurement(3,0) = transform.quaternion.x;
        measurement(4,0) = transform.quaternion.y;
        measurement(5,0) = transform.quaternion.z;
        measurement(6,0) = transform.quaternion.w;

        for(unsigned i = 0; i < 3; i++){
            measurement_error(i,0) = 5.0f*error;
        }

        for(unsigned i = 3; i < 7; i++){
            measurement_error(i,0) = error;
        }

        pose_filter.performMeasurementUpdate(measurement, measurement_error);

        Matrix<float,7,1> filtered_pose = pose_filter.getCurrentPose();
        object_transform.shift = Vector3D(filtered_pose(0,0),filtered_pose(1,0),filtered_pose(2,0));

        object_transform.quaternion = Quaternion(filtered_pose(3,0), filtered_pose(4,0), filtered_pose(5,0), filtered_pose(6,0));
        object_transform.quaternion.normalise();
        object_transform.mat = object_transform.quaternion.toMatrix();

        last_normal_vector = snapshot_ag[best_index].first;
        //last_ag_index = best_index;
        //last_ag_result = best_result;
        have_last_result = true;

        frames_since_last_seen = 0;
        num_features_matched = best_result;
        return true;
    }

    frames_since_last_seen++;
    return false;
}


void ObjectSceneMatch::reset(void){
    pose_filter.reset();
    best_fit.reset();
}
