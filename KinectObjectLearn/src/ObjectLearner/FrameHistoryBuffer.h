
#ifndef _FrameHistoryBuffer_H_
#define _FrameHistoryBuffer_H_

#include "../Features/SIFTFeature3D.h"
#include "../KinectCamera/KinectCamera.h"

#include <vector>
#include <list>
#include <map>

struct FrameHistoryElem {
    KinectCamera::CorrelatedImage correlated_frame;
    std::vector<SIFTFeature3D> features;
    std::vector<float> arm_joints;
    unsigned frame_num;
};

class FrameHistoryBuffer {
  public:
    FrameHistoryBuffer(unsigned max_size);
    ~FrameHistoryBuffer();

    void addToBuffer(const KinectCamera::CorrelatedImage &correlated_frame,
                     const std::vector<SIFTFeature3D> &features,
                     const std::vector<float> &arm_joints,
                     unsigned frame_num);

    bool getHistoryFrame(KinectCamera::CorrelatedImage &correlated_frame,
                         std::vector<SIFTFeature3D> &features,
                         unsigned frame_num);

    bool getHistoryJoints(std::vector<float> &arm_joints, unsigned frame_num);

  private:

    unsigned msize;

    // List of frame numbers in order.
    std::list<unsigned> history_queue;

    // Map from frame number to frame element.
    std::map<unsigned, FrameHistoryElem*> history_frames;

};

#endif

