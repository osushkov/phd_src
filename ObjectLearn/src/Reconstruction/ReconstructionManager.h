/*
 * ReconstructionManager.h
 *
 *  Created on: 03/07/2009
 *      Author: osushkov
 */

#ifndef RECONSTRUCTIONMANAGER_H_
#define RECONSTRUCTIONMANAGER_H_

#include "ReconstructionFrame.h"
#include "../Features/StereoFeature.h"
#include "../Util/Matrix.h"
#include "../Util/Octree.h"
#include "../Visualisation/PointCloudRenderObject.h"
#include "BestFit.h"

#include <string>
#include <map>

struct ModelFrame {
    std::vector<StereoFeature> features;
    Vector3D view_direction;
};

class ReconstructionManager {
  public:
    ReconstructionManager(std::string object_name);
    ~ReconstructionManager();

    void submitFeatures(const std::vector<StereoFeature> &object_features,
                        const std::vector<StereoFeature> &arm_features);

    void submitFeatures(const std::vector<StereoFeature> &object_features,
                        const std::vector<StereoFeature> &arm_features,
                        unsigned frame_id);

    std::vector<ModelFrame> reconstruct(void);
    static std::vector<Vector3D> getFeaturePositions(const std::vector<StereoFeature> &features);


  private:

    std::string object_name;
    unsigned cur_frame_id, cur_feature_id;

    std::map<unsigned, ReconstructionFrame> reconstruction_frames;

    PointCloudRenderObject *point_cloud, *arm_point_cloud;
    BestFit best_fit;


    unsigned findNumberMatchingFeatures(const ReconstructionFrame &frame1,
                                        const ReconstructionFrame &frame2);

    Vector3D applyTransform(const Transform &t, const Vector3D &vec);

    void linkFrames(void);
    void linkFeatures(void);
    void reconstructFrame(ReconstructionFrame &frame);

    std::vector<BestFitFeaturePair>
    linkFrameFeatures(ReconstructionFrame &src,
                      ReconstructionFrame &dst,
                      std::vector<unsigned> &linked_src_features);

    void applyFrameTransform(ReconstructionFrame &frame);

    std::vector<ModelFrame> generateModelFrames(void);
    Octree<StereoFeature>* buildOctree(const std::vector<StereoFeature> &features);

    bool isObjectFeatureByWeight(const StereoFeature &f,
                                 Octree<StereoFeature> *object_features_octree,
                                 Octree<StereoFeature> *arm_features_octree);

    bool isObjectFeatureByWeight(const StereoFeature &f,
                                 const std::vector<StereoFeature> &object_features,
                                 const std::vector<StereoFeature> &arm_features);

    bool isObjectFeatureArmLineTest(const StereoFeature &f,
                                    const std::vector<std::pair<Vector3D,Vector3D> > &arm_lines);

    Vector3D findRotationCentre(std::vector<BestFitFeaturePair> pos_pairs);

};


#endif /* RECONSTRUCTIONMANAGER_H_ */
