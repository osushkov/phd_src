/*
 * ObjectSnapshotDB.h
 *
 *  Created on: 10/08/2009
 *      Author: osushkov
 */

#ifndef OBJECTSNAPSHOTDB_H_
#define OBJECTSNAPSHOTDB_H_

#include "../SIFTFeature3D.h"
#include "ObjectSnapshot.h"

#include <string>
#include <vector>
#include <set>


struct SceneMatchResult {
    ObjectSnapshot *matched_snapshot;
    std::vector<SnapshotMatchResult> feature_matches;
    Transform approximate_transform;
    float match_score;
};

class ObjectSnapshotDB {
  public:
    static float getSIFTMatchScore(float sift_distance);
    static ObjectSnapshotDB* getArmSnapshotDB(void);
    static std::set<unsigned> getMatchedFeatureIndexes(const std::vector<SIFTFeature3D> &features,
                                                       const SceneMatchResult &match);

    ObjectSnapshot* createNewSnapshot(std::string name);
    ObjectSnapshot* createNewSnapshot(std::string name, Vector3D to_camera); 

    ObjectSnapshotDB(std::string object_name);
    ~ObjectSnapshotDB();

    bool load(std::string filename);
    bool save(std::string filename);

    SceneMatchResult matchScene(const std::vector<SIFTFeature3D> &scene_features,
                                bool hi_accuracy=true);

    SceneMatchResult matchScene(const std::vector<SIFTFeature3D> &scene_features,
                                Vector3D view_vector,
                                bool hi_accuracy=true);

    std::vector<SceneMatchResult> matchSceneMany(const std::vector<SIFTFeature3D> &scene_features,
                                                 float match_threshold, 
                                                 bool hi_accuracy=true);

    std::vector<SceneMatchResult> matchSceneMany(const std::vector<SIFTFeature3D> &scene_features,
                                                 Vector3D view_vector, 
                                                 float match_threshold, 
                                                 bool hi_accuracy=true);

    void addSnapshot(ObjectSnapshot *snapshot);

    ObjectSnapshot* getSnapshotById(unsigned id);
    std::string getObjectName(){
        return object_name;
    }

    std::vector<ObjectSnapshot*> getSnapshots(){
        return object_snapshots;
    }

  private:
    std::string object_name;

    std::vector<ObjectSnapshot*> object_snapshots;
    unsigned cur_snapshot_id;

    SceneMatchResult matchSceneWithSnapshots(const std::vector<SIFTFeature3D> &scene_features,
                                             std::vector<ObjectSnapshot*> snapshots_pool,
                                             bool hi_accuracy);

    std::vector<SceneMatchResult> matchSceneWithManySnapshots(const std::vector<SIFTFeature3D> &scene_features,
                                                              std::vector<ObjectSnapshot*> snapshots_pool,
                                                              float match_threshold,
                                                              bool hi_accuracy);
};


#endif /* OBJECTSNAPSHOTDB_H_ */
