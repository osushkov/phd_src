/*
 * ObjectSceneMatch.cpp
 *
 *  Created on: 10/12/2009
 *      Author: osushkov
 */

#include "ObjectSceneMatch.h"
#include "Util/Timer.h"
#include "Util/Matrix.h"
#include "Util/Common.h"

ObjectSceneMatch::ObjectSceneMatch(std::string object_name,
                                   const std::vector<SnapshotFrame> &object_frames) :
                                       object_name(object_name),
                                       frames_since_last_seen(-1),
                                       have_last_result(false) {

    assert(object_frames.size() > 0);
    for(unsigned i = 0; i < object_frames.size(); i++){
        ObjectSnapshot *new_snapshot = new ObjectSnapshot(object_name, i);
        new_snapshot->addFeatures(object_frames[i].object_features, std::vector<SurfacePoint>());

        std::pair<Vector3D,ObjectSnapshot*> new_entry(object_frames[i].to_camera, new_snapshot);
        snapshot_ag.push_back(new_entry);
    }
}


ObjectSceneMatch::~ObjectSceneMatch() {

}

bool ObjectSceneMatch::sceneMatch(const std::vector<SIFTFeature3D> &scene_features,
                                  Transform &object_transform,
                                  std::vector<unsigned> &matched_scene_features){


    //pose_filter.performMotionUpdate();

    float best_result = 0.0f;
    unsigned best_index = 0;
    std::vector<SnapshotMatchResult> best_matches;
    Transform best_approx_transform;

    Util::Timer timer;
    timer.start();

    float max_angle_from_last = 180.0f*M_PI/180.0f;
/*    if(frames_since_last_seen < 20.0f){
        max_angle_from_last = (frames_since_last_seen+1)*15.0f*M_PI/180.0f;
    }
*/
    unsigned num_checks = 0;

    for(unsigned i = 0; i < snapshot_ag.size(); i++){
        float angle = acosf(last_normal_vector.dotProduct(snapshot_ag[i].first));
        if(have_last_result && angle > max_angle_from_last){
            //continue;
        }

        std::vector<SnapshotMatchResult> matches;
        num_checks++;

        Transform approx_transform;
        float r = snapshot_ag[i].second->matchScene(scene_features, matches, approx_transform, true);
        if(r > best_result){
            best_result = r;
            best_index = i;
            best_matches = matches;
            best_approx_transform = approx_transform;
        }

        if(best_result > 20){
            //break;
        }

        //std::cout << i << "/" << snapshot_ag.size() << std::endl;
    }

    timer.stop();

    std::cout << "nc: " << num_checks << " " << best_result << " "
              << timer.getNumElapsedSeconds()*1000.0f << std::endl;


    if(best_result >= 3.0f){
        std::vector<BestFitFeaturePair> pos_pairs;
        std::vector<SIFTFeature3D> snapshot_features = snapshot_ag[best_index].second->getFeatures();
        Transform transform;

        std::vector< std::pair<Vector2D,Vector3D> > perspective_pairs;
        matched_scene_features.clear();
        for (unsigned i = 0; i < best_matches.size(); i++) {
            BestFitFeaturePair bffp;
            bffp.fpos0 = scene_features[best_matches[i].scene_index].position;
            bffp.fpos1 = snapshot_features[best_matches[i].snapshot_index].position;
            bffp.pmatch = best_matches[i].pmatch;
            pos_pairs.push_back(bffp);


            matched_scene_features.push_back(best_matches[i].scene_index);

            std::pair<Vector2D,Vector3D> new_perspective_pair;
            new_perspective_pair.first = Vector2D(scene_features[best_matches[i].scene_index].sift_feature.x,
                                                  scene_features[best_matches[i].scene_index].sift_feature.y);
            new_perspective_pair.second = snapshot_features[best_matches[i].snapshot_index].position;
            perspective_pairs.push_back(new_perspective_pair);
        }


        //Transform cam_transform = Common::cameraToWorldSpaceTransform(0.0f, -65.0f, 0.0);
        //transform.shift = Common::cameraPointToWorldSpace(transform.shift, 0.0f, -65.0f, 0.0);
        //transform.shift = cam_transform.mat*transform.shift + cam_transform.shift;
        //transform.mat = cam_transform.mat*transform.mat;

        best_fit.setHint(best_approx_transform);
        float error = best_fit.calculateBestFit(pos_pairs, transform, false);
        std::cout << "error: " << error << " " << pos_pairs.size() << std::endl;

        if(error > 0.2f){
            //best_fit.reset();
            //frames_since_last_seen++;
            return false;
        }

        object_transform = transform;
        object_transform.mat = Common::quaternionsToMatrix(object_transform.quaternions);
/*
        Matrix<float,7,1> measurement;
        Matrix<float,7,1> measurement_error;

        measurement(0,0) = transform.shift.x;
        measurement(1,0) = transform.shift.y;
        measurement(2,0) = transform.shift.z;

        for(unsigned i = 0; i < 4; i++){
            measurement(i+3,0) = transform.quaternions[i];
        }

        for(unsigned i = 0; i < 3; i++){
            measurement_error(i,0) = 5.0f*error;
        }

        for(unsigned i = 3; i < 7; i++){
            measurement_error(i,0) = error;
        }

        pose_filter.performMeasurementUpdate(measurement, measurement_error);

        Matrix<float,7,1> filtered_pose = pose_filter.getCurrentPose();
        object_transform.shift = Vector3D(filtered_pose(0,0),filtered_pose(1,0),filtered_pose(2,0));

        object_transform.quaternions.clear();
        object_transform.quaternions.push_back(filtered_pose(3,0));
        object_transform.quaternions.push_back(filtered_pose(4,0));
        object_transform.quaternions.push_back(filtered_pose(5,0));
        object_transform.quaternions.push_back(filtered_pose(6,0));
        Common::normaliseVector(object_transform.quaternions);

        object_transform.mat = Common::quaternionsToMatrix(object_transform.quaternions);
*/
        //last_normal_vector = snapshot_ag[best_index].first;
        //last_ag_index = best_index;
        //last_ag_result = best_result;
        //have_last_result = true;

        //frames_since_last_seen = 0;
        return true;
    }

    //frames_since_last_seen++;
    return false;
}

bool ObjectSceneMatch::sceneMatch(const std::vector<SIFTFeature3D> &scene_features,
                                            Transform &object_transform){
    std::vector<unsigned> tmp;
    return sceneMatch(scene_features, object_transform, tmp);
}

void ObjectSceneMatch::reset(void){
    best_fit.reset();
}
