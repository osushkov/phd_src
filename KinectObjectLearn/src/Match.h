
#ifndef _Match_H_
#define _Match_H_

#include <vector>
#include "KinectCamera/KinectCamera.h"
#include "Features/SIFTFeature3D.h"
#include "Features/FeatureMemory/ObjectSnapshot.h"
#include "Util/Transform.h"

struct MatchData {
    std::vector<KinectCamera::DepthPixel> src_pixels, dst_pixels;
    std::vector<SIFTFeature3D> src_features, dst_features;
    std::vector<SnapshotMatchResult> feature_matches;
    Transform approx_transform;
};

class Match {
  public:
    Match();
    ~Match();

    Transform doMatch(MatchData &match_data, float &err);
    Transform doMatchApprox(MatchData &match_data, float &err);
    Transform refine(std::vector<Vector3D> src_points, std::vector<Vector3D> dst_points);

  private:
    void filterPoints(std::vector<Vector3D> &src_points, std::vector<Vector3D> &dst_points);
    
    bool areUniquePositions(const SIFTFeature3D &var1, const SIFTFeature3D &var2);
    Transform buildTransform(std::vector<Vector3D> src_triplet, std::vector<Vector3D> dst_triplet);

    float evalTransform(Transform t, MatchData &match_data);
    Vector3D applyTransform(Transform t, Vector3D p);
};

#endif
