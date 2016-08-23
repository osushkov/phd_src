/*
 * ObjectSnapshotDB.h
 *
 *  Created on: 10/08/2009
 *      Author: osushkov
 */

#ifndef OBJECTSNAPSHOTDB_H_
#define OBJECTSNAPSHOTDB_H_

#include "../StereoFeature.h"
#include "ObjectSnapshot.h"

#include <string>
#include <vector>
#include <set>


struct SceneMatchResult {
    std::set<unsigned> matched_scene_features;
};

class ObjectSnapshotDB {
  public:
    static float getSIFTMatchScore(float sift_distance);
    static ObjectSnapshotDB* getArmSnapshotDB(void);

    ObjectSnapshotDB(std::string object_name);
    ~ObjectSnapshotDB();

    bool load(std::string filename);
    bool save(std::string filename);

    SceneMatchResult matchScene(const std::vector<StereoFeature> &scene_features,
                                bool hi_accuracy=true);

    void addSnapshot(const std::vector<StereoFeature> &features);

    ObjectSnapshot* getSnapshotById(unsigned id);

private:
    std::string object_name;

    std::vector<ObjectSnapshot> object_snapshots;
    unsigned cur_snapshot_id;

};


#endif /* OBJECTSNAPSHOTDB_H_ */
