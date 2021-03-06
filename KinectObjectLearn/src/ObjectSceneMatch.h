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

#include "Features/SIFTFeature3D.h"
#include "Features/FeatureMemory/ObjectSnapshot.h"
#include "Reconstruction/ReconstructionManager.h"
#include "Reconstruction/ReconstructionFrame.h"
#include "Reconstruction/SuperQuadric.h"
#include "Reconstruction/BestFit.h"
#include "Reconstruction/SnapshotFrames.h"
#include "ObjectPoseFilter.h"
#include "SceneBestFit.h"

class ObjectSceneMatch {
  public:
    ObjectSceneMatch(std::string object_name, const std::vector<SnapshotFrame> &object_frames);
    ~ObjectSceneMatch();

    bool sceneMatch(const std::vector<SIFTFeature3D> &scene_features,
                    Transform &object_transform);

    bool sceneMatch(const std::vector<SIFTFeature3D> &scene_features,
                    Transform &object_transform,
                    std::vector<unsigned> &matched_scene_features);

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
