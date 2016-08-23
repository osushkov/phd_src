/*
 * ObjectSnapshotDBMono.h
 *
 *  Created on: 25/08/2009
 *      Author: osushkov
 */

#ifndef OBJECTSNAPSHOTDBMONO_H_
#define OBJECTSNAPSHOTDBMONO_H_

#include "../SIFT/sift.h"
#include "ObjectSnapshotMono.h"

#include <string>
#include <vector>
#include <set>


class ObjectSnapshotDBMono {
  public:
    static float getSIFTMatchScore(float sift_distance);

    ObjectSnapshotDBMono();
    ~ObjectSnapshotDBMono();

    void setParams(const std::vector<float> &params);

    void load(std::string filename);
    void save(std::string filename);

    std::set<unsigned> matchScene(const std::vector<feature> &scene_features,
                                  std::string object="");

    void addSnapshot(std::string object_name, const std::vector<feature> &features);

    ObjectSnapshotMono* getSnapshotById(unsigned id);

    unsigned getNumFeatures(void);

private:

    std::vector<ObjectSnapshotMono *> object_snapshots;
    unsigned cur_snapshot_id;

    float param_max_pixel_match_dist;
    float param_min_pixel_sift_match;
    float param_min_basis_pmatch;
    float param_distance_score_sd;
    float param_orientation_score_sd;

};
#endif /* OBJECTSNAPSHOTDBMONO_H_ */
