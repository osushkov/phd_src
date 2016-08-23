
#ifndef _SnapshotFrames_H_
#define _SnapshotFrames_H_

#include "../KinectCamera/KinectCamera.h"
#include "../Features/SIFTFeature3D.h"
#include <vector>
#include <string>
#include <iostream>

struct SnapshotFrame {
    std::vector<SIFTFeature3D> arm_features, object_features;
    std::vector<KinectCamera::DepthPixel> arm_pixels, object_pixels;
    Vector3D to_camera;

    void writeOutFrame(std::ostream &out_file);
    void writeOutFrameFeatures(std::ostream &out_file);
    static SnapshotFrame readFrame(std::istream &in_file);
    static SnapshotFrame readFrameFeatures(std::istream &in_file);

};

class SnapshotFrames {
  public:
    SnapshotFrames();
    ~SnapshotFrames();

    void addFrame(const SnapshotFrame &frame);
    std::vector<SnapshotFrame> getFrames(void);

    void save(std::string filename);
    bool load(std::string filename);

  private:

    std::vector<SnapshotFrame> snapshot_frames;
    SnapshotFrame readInFrame(std::istream &in_file);
};

#endif
