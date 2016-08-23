/*
 * ObjectSnapshot.h
 *
 *  Created on: 10/08/2009
 *      Author: osushkov
 */

#ifndef OBJECTSNAPSHOT_H_
#define OBJECTSNAPSHOT_H_

#include "../SIFTFeature3D.h"
#include "../../Util/Transform.h"
#include "../../Util/Octree.h"
#include "../../Util/Semaphore.h"

#include <string>
#include <vector>
#include <iostream>
#include <set>

struct FeatureMatchPair {
    unsigned snap_feature_index;
    unsigned scene_feature_index;
    float sift_distance;
    float sift_pmatch;
};

struct BasisTriplet {
    std::vector<unsigned> point_index; // indexes into the match_pairs array
    std::vector<std::pair<SIFTFeature3D,SIFTFeature3D> > point_feature; // snapshot-scene features
    float p_match; // probability of a match.
};

struct SnapshotMatchResult {
    unsigned scene_index; // scene (dst)
    unsigned snapshot_index; // snapshot (src);
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

struct SurfacePoint {
    Vector3D pos;
    Vector3D color;
};

class ObjectSnapshot {
  public:

    ObjectSnapshot(std::string name, unsigned id);
    ObjectSnapshot(std::string name, unsigned id, Vector3D to_camera);
    ObjectSnapshot(const ObjectSnapshot &that);

    ~ObjectSnapshot();

    void load(std::istream &input_stream);
    void save(std::ostream &output_stream);

    unsigned getId(void) const { return id; }
    inline std::string getName(void) const { return name; }
    inline std::vector<SIFTFeature3D> getFeatures(void) const { return snapshot_features; }
    inline std::vector<SurfacePoint> getSurfacePoints(void) const { return surface_points; }

    float matchScene(const std::vector<SIFTFeature3D> &scene_features,
                     std::vector<SnapshotMatchResult> &matches,
                     Transform &approx_transform,
                     bool hi_accuracy=true);

    //float getBestBasisTriplet(const std::vector<StereoFeature> &scene_features,
    //                          std::vector<std::pair<unsigned,unsigned> > &basis);

    void addFeatures(const std::vector<SIFTFeature3D> &new_features, const std::vector<SurfacePoint> &new_surface_points);
    void addSurfacePoints(const std::vector<SurfacePoint> &new_surface_points);
    void clearFeatures(void);
    void clearSurfacePoints(void);

    inline Vector3D getToCamera() const {
        return to_camera;
    }

    inline bool haveToCamera() const {
        return have_to_camera;
    }

  private:
    unsigned id;
    std::string name;

    Vector3D to_camera;
    bool have_to_camera;

    std::vector<SIFTFeature3D> snapshot_features;
    std::vector<SurfacePoint> surface_points;

    ObjectSnapshotParams params;


    std::vector<float> buildDistanceMatrix(const std::vector<SIFTFeature3D> &scene_features);

    std::vector<FeatureMatchPair>
    buildMatchPairs(const std::vector<SIFTFeature3D> &scene_features,
                    std::vector<float> &distance_matrix);

    float queryDistanceMatrix(unsigned snap_index, unsigned scene_index,
                              std::vector<float> &distance_matrix, unsigned width);

    float tryMatchScene(const BasisTriplet &basis,
                        const std::vector<SIFTFeature3D> &scene_features,
                        std::vector<SnapshotMatchResult> &matches,
                        Transform &approx_transform,
                        std::vector<float> &distance_matrix);

    std::vector<BasisTriplet>
    findBasisTriplets(const std::vector<SIFTFeature3D> &scene_features,
                      const std::vector<FeatureMatchPair> &match_pairs);

    bool areUniquePositions(const SIFTFeature3D &var1, const SIFTFeature3D &var2);
    float evalTripletPMatch(BasisTriplet &triplet, const std::vector<FeatureMatchPair> &match_pairs);
    float evalTripletSpacialSize(BasisTriplet &triplet);

    void buildBasisSet(const BasisTriplet &basis,
                       std::vector<Vector3D> &snapshot_basis_set,
                       std::vector<Vector3D> &scene_basis_set);

    std::vector<float> featureAngleMatchScore(const feature &a, const feature &b);
    float calculateOrientationOffset(const BasisTriplet &triplet);

    void setParams(bool hi_accuracy);
    Octree<SIFTFeature3D>* buildSnapshotOctree(const std::vector<SIFTFeature3D> &stereo_features);
};


#endif /* OBJECTSNAPSHOT_H_ */
