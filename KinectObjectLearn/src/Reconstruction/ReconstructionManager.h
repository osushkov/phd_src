/*
 * ReconstructionManager.h
 *
 *  Created on: 03/07/2009
 *      Author: osushkov
 */

#ifndef RECONSTRUCTIONMANAGER_H_
#define RECONSTRUCTIONMANAGER_H_

#include "../Features/SIFTFeature3D.h"
#include "../Util/Matrix.h"
#include "../Util/Octree.h"
#include "../Visualisation/PointCloudRenderObject.h"
#include "SnapshotFrames.h"
#include "BestFit.h"

#include <string>
#include <map>

struct ModelFrame {
    SnapshotFrame frame;
    ObjectSnapshot *snapshot;

    std::vector<SIFTFeature3D> features;
    Vector3D view_direction;
    bool glued;
    bool behind;
    bool is_valid;
};

struct ModelMatchData {
    float best_match_score;
    Transform best_transform;
    std::vector<float> all_match_scores;
    std::vector<std::vector<SnapshotMatchResult> > all_matches;
    std::vector<unsigned> all_frame_features_index;
};

class ReconstructionManager {
  public:
    ReconstructionManager(std::string object_name);
    ReconstructionManager();
    ~ReconstructionManager();

    std::vector<SnapshotFrame> reconstruct(const std::vector<SnapshotFrame> &snapshot_frames);
    void stitchTogether(std::vector<SnapshotFrame> &frame0, std::vector<SnapshotFrame> &frame1);

  private:

    std::string object_name;
    std::map<unsigned, ModelMatchData> frame_match_data;

    PointCloudRenderObject *object_point_cloud, *arm_point_cloud;
    std::vector<ModelFrame> model_frames;

    std::vector<Vector3D> getAllObjectPixels(const std::vector<SnapshotFrame> &frames, int except = -1);
    std::vector<Vector3D> getAllObjectPixels(const SnapshotFrame &frame);

    void cleanUpFrames(std::vector<SnapshotFrame> &frames);
    void reconstructFrame(unsigned index, float iter_frac);
    void genModelFrames(const std::vector<SnapshotFrame> &snapshot_frames);
    ModelMatchData genModelMatchData(unsigned mode_frame_index);
    void addFrameToPointClouds(const ModelFrame &frame);
    void applyTransformToFrame(Transform t, ModelFrame &frame);
    void applyTransformToSnapshotFrame(Transform t, SnapshotFrame &frame);
    void insertObjectPixels(std::vector<Vector3D> &dst, const ModelFrame &frame);
    std::vector<Vector3D> getAllReconstructionPixels(unsigned except_index, float iter_frac);
    std::vector<Vector3D> getReconstructionPixels(const ModelFrame &frame, float iter_frac);
    void centraliseModelFrames(void);
    void filterModelFrames(void);
};


#endif /* RECONSTRUCTIONMANAGER_H_ */
