/*
 * ObjectSnapshotMono.h
 *
 *  Created on: 25/08/2009
 *      Author: osushkov
 */

#ifndef OBJECTSNAPSHOTMONO_H_
#define OBJECTSNAPSHOTMONO_H_

#include "../SIFT/sift.h"
#include "../../Util/Matrix.h"

#include <string>
#include <vector>
#include <iostream>


struct FeatureMatchPairMono {
    unsigned snap_feature_index;
    unsigned scene_feature_index;
    float sift_distance;
    float sift_pmatch;
};

struct BasisPair {
    std::vector<unsigned> point_index; // indexes into the match_pairs array
    std::vector<std::pair<feature,feature> > point_feature; // snapshot-scene features
    float p_match; // probability of a match.
};


class ObjectSnapshotMono {
  public:
    ObjectSnapshotMono(std::string name, unsigned id,
                       float max_pixel_match_dist,
                       float min_pixel_sift_match,
                       float min_basis_pmatch,
                       float distance_score_sd,
                       float orientation_score_sd);

    ~ObjectSnapshotMono();

    void load(std::istream &input_stream);
    void save(std::ostream &output_stream);

    unsigned getId(void) const { return id; }
    inline std::string getName(void) const { return name; }
    inline std::vector<feature> getFeatures(void) const { return snapshot_features; }

    float matchScene(const std::vector<feature> &scene_features,
                     std::vector<unsigned> &matched_indexes);

    void addFeatures(const std::vector<feature> &new_features);

  private:
    unsigned id;
    std::string name;
    std::vector<feature> snapshot_features;
    unsigned matched_features;

    std::vector<FeatureMatchPairMono> match_pairs;

    std::vector<float> distance_matrix;
    unsigned width;

    const float max_pixel_match_dist;
    const float min_pixel_sift_match;
    const float min_basis_pmatch;
    const float distance_score_sd;
    const float orientation_score_sd;


    void buildDistanceMatrix(const std::vector<feature> &scene_features);
    float queryDistanceMatrix(unsigned snap_index, unsigned scene_index);

    void buildMatchPairs(const std::vector<feature> &scene_features);

    float tryMatchScene(const BasisPair &basis,
                        const std::vector<feature> &scene_features,
                        std::vector<unsigned> &matched_indexes);

    std::vector<BasisPair>
    findBasisPairs(const std::vector<feature> &scene_features);

    bool areUniquePositions(const feature &var1, const feature &var2);
    float evalPairPMatch(const BasisPair &triplet);
    float evalPairSpacialSize(const BasisPair &triplet);

    float featureEuclideanDistance(const feature &a, const feature &b);
    Matrix2 buildTransformMatrix(const BasisPair &basis);

    std::vector<float> featureAngleMatchScore(const feature &a, const feature &b);

};


#endif /* OBJECTSNAPSHOTMONO_H_ */
