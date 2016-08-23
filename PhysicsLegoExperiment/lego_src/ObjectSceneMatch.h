/*
 * ObjectSceneMatch.h
 *
 *  Created on: 10/12/2009
 *      Author: osushkov
 */

#ifndef OBJECTSCENEMATCH_H_
#define OBJECTSCENEMATCH_H_

#include <vector>
#include <string>

#include "Features/StereoFeature.h"
#include "Features/FeatureMemory/ObjectSnapshot.h"
#include "ObjectPoseFilter.h"
#include "SceneBestFit.h"

class ObjectSceneMatch {
  public:
    ObjectSceneMatch(std::string object_name, const std::vector<ModelFrame> &object_frames);
    ~ObjectSceneMatch();

    bool sceneMatch(const std::vector<StereoFeature> &scene_features,
                    Transform &object_transform,
                    unsigned &num_features_matched);

    void reset(void);

  private:
    std::string object_name;

    unsigned frames_since_last_seen;
    Vector3D last_normal_vector;
    unsigned last_ag_index;
    float last_ag_result;
    bool have_last_result;

    // Snapshot aspect graph, each snapshot of features is paired with a normal
    // vector which is the vector from the object to the camera for the corresponding
    // view of features.
    std::vector<std::pair<Vector3D,ObjectSnapshot*> > snapshot_ag;
    SceneBestFit best_fit;

    ObjectPoseFilter pose_filter;

    void buildAspectGraph(void);
};


#endif /* OBJECTSCENEMATCH_H_ */
