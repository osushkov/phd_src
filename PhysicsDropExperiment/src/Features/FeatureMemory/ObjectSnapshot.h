/*
 * ObjectSnapshot.h
 *
 *  Created on: 10/08/2009
 *      Author: osushkov
 */

#ifndef OBJECTSNAPSHOT_H_
#define OBJECTSNAPSHOT_H_

#include "../StereoFeature.h"
#include "../../Util/Transform.h"
#include "../../Util/Octree.h"

#include <string>
#include <vector>
#include <iostream>

struct FeatureMatchPair {
    unsigned snap_feature_index;
    unsigned scene_feature_index;
    float sift_distance;
    float sift_pmatch;
};

struct BasisTriplet {
    std::vector<unsigned> point_index; // indexes into the match_pairs array
    std::vector<std::pair<StereoFeature,StereoFeature> > point_feature; // snapshot-scene features
    float p_match; // probability of a match.
};

struct SnapshotMatchResult {
    unsigned index0; // scene
    unsigned index1; // snapshot;
    float pmatch;
};

struct ObjectSnapshotParams {
    int num_triplets_check;

    float max_fpos_diff;
    float max_fori_diff;
    float min_fpmatch;

    int max_triplets;
    int max_triplets_check;

    float min_triplet_score;
    float triplet_dist_sd;
    float triplet_ori_sd;
};

class ObjectSnapshot {
  public:
    ObjectSnapshot(std::string name, unsigned id);
    ~ObjectSnapshot();

    void load(std::istream &input_stream);
    void save(std::ostream &output_stream);

    unsigned getId(void) const { return id; }
    inline std::string getName(void) const { return name; }
    inline std::vector<StereoFeature> getFeatures(void) const { return snapshot_features; }

    float matchScene(const std::vector<StereoFeature> &scene_features,
                     std::vector<SnapshotMatchResult> &matches,
                     bool hi_accuracy=true);

    float matchScene(const std::vector<StereoFeature> &scene_features,
                     std::vector<SnapshotMatchResult> &matches,
                     Transform &approx_transform,
                     bool hi_accuracy=true);

    //float getBestBasisTriplet(const std::vector<StereoFeature> &scene_features,
    //                          std::vector<std::pair<unsigned,unsigned> > &basis);

    void addFeatures(const std::vector<StereoFeature> &new_features);
    void clearFeatures(void);

  private:
    unsigned id;
    std::string name;
    std::vector<StereoFeature> snapshot_features;
    unsigned matched_features;

    std::vector<FeatureMatchPair> match_pairs;

    std::vector<float> distance_matrix;
    unsigned width;

    ObjectSnapshotParams params;


    void buildDistanceMatrix(const std::vector<StereoFeature> &scene_features);
    void buildMatchPairs(const std::vector<StereoFeature> &scene_features);

    float queryDistanceMatrix(unsigned snap_index, unsigned scene_index);
    float tryMatchScene(const BasisTriplet &basis,
                        const std::vector<StereoFeature> &scene_features,
                        std::vector<SnapshotMatchResult> &matches,
                        Transform &approx_transform);

    std::vector<BasisTriplet>
    findBasisTriplets(const std::vector<StereoFeature> &scene_features);

    bool areUniquePositions(const StereoFeature &var1, const StereoFeature &var2);
    float evalTripletPMatch(BasisTriplet &triplet);
    float evalTripletSpacialSize(BasisTriplet &triplet);

    void buildBasisSet(const BasisTriplet &basis,
                       std::vector<Vector3D> &snapshot_basis_set,
                       std::vector<Vector3D> &scene_basis_set);

    std::vector<float> featureAngleMatchScore(const feature &a, const feature &b);
    float calculateOrientationOffset(const BasisTriplet &triplet);

    void setParams(bool hi_accuracy);
    Octree<StereoFeature>* buildSnapshotOctree(const std::vector<StereoFeature> &stereo_features);
};


#endif /* OBJECTSNAPSHOT_H_ */
