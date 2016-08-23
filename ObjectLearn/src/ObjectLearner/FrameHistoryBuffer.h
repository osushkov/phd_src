
#ifndef _FrameHistoryBuffer_H_
#define _FrameHistoryBuffer_H_

#include "../Features/StereoFeature.h"

#include <vector>
#include <list>
#include <map>

struct FrameHistoryElem {
    std::vector<unsigned char> left_frame, right_frame;
    std::vector<StereoFeature> features;
    std::vector<float> arm_joints;
    unsigned frame_num;
};

class FrameHistoryBuffer {
  public:
    FrameHistoryBuffer(unsigned max_size);
    ~FrameHistoryBuffer();

    void addToBuffer(const std::vector<unsigned char> &left_frame,
                     const std::vector<unsigned char> &right_frame,
                     const std::vector<StereoFeature> &features,
                     const std::vector<float> &arm_joints,
                     unsigned frame_num);

    bool getHistoryFrame(std::vector<unsigned char> &left_frame,
                         std::vector<unsigned char> &right_frame,
                         std::vector<StereoFeature> &features,
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

