/*
 * RawPointCloud.h
 *
 *  Created on: 01/11/2009
 *      Author: osushkov
 */

#ifndef RAWPOINTCLOUD_H_
#define RAWPOINTCLOUD_H_

#include "../Features/StereoFeature.h"
#include "../Features/FeatureMemory/ObjectSnapshotDB.h"
#include "../CameraController.h"

#include <vector>
#include <string>


struct RawPointCloudFrame {
    std::vector<StereoFeature> object_features;
    std::vector<StereoFeature> arm_features;
};

class RawPointCloud {
  public:
    RawPointCloud(std::string object_name);
    ~RawPointCloud();

    bool load(std::string filename);
    bool save(std::string filename);

    void generate(ObjectSnapshotDB *object_snapshot_db, std::string video_path);
    std::vector<RawPointCloudFrame> getPointCloudFrames(void) const;


  private:
    std::string object_name;
    std::vector<RawPointCloudFrame> point_cloud_frames;

    void renderDetectedPoints(CameraController &camera_controller,
                              RawPointCloudFrame &point_cloud_frame);

    void partitionFeatureSet(const std::vector<StereoFeature> &start_features,
                             float sd_threshold,
                             std::vector<StereoFeature> &in_features,
                             std::vector<StereoFeature> &out_features);

};

#endif /* RAWPOINTCLOUD_H_ */
