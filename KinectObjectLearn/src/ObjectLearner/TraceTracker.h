
#ifndef _TraceTracker_H_
#define _TraceTracker_H_

#include "../Features/SIFTFeature3D.h"
#include <vector>
#include <map>

struct TraceNode {
    SIFTFeature3D feature;
    unsigned frame; // in which frame was this feature seen?
};

struct Trace {
    int tag;
    std::vector<TraceNode> nodes; // nodes in order of oldest to newest
};

class TraceTracker {
  public:
    TraceTracker();
    ~TraceTracker();

    void submitStereoFeatures(std::vector<SIFTFeature3D> features,
                              std::vector<float> arm_joints,
                              unsigned cur_frame);

    std::vector<Trace> getActiveTraces(void);

    bool shouldCorrelate(unsigned &frame_start, std::vector<SIFTFeature3D> &features_start,
                         unsigned &frame_end, std::vector<SIFTFeature3D> &features_end,
                         std::vector<Trace> &paths,
                         unsigned cur_frame);

    std::vector<SIFTFeature3D> getFrameFeatures(unsigned frame_num);
    bool findClosestFeature(SIFTFeature3D feature, std::vector<SIFTFeature3D> &all_features,
                            unsigned &index);

    void printBins(void);

  private:

    unsigned last_snapshot_frame;
    unsigned latest_frame;
    std::vector<Trace> all_traces; // history of ALL traces.
    std::vector<Trace> dummy_arm_traces;

    // a list of all traces which can still be appended to
    std::vector<unsigned> cur_active_traces_ref;
    std::vector<unsigned> cur_active_arm_traces_ref;

    std::map<unsigned, std::vector<SIFTFeature3D> > frame_features;
    std::map<unsigned, std::vector<float> > frame_arm_joints;


    // Insert the given feature into the set of currently active traces, if
    // no suitable active traces are found to include this feature, a new
    // trace will be created.
    void insertFeature(SIFTFeature3D feature, unsigned cur_frame);

    // Checks whether two five stereo features are almost certainly the
    // same feature, ie: both left and right SIFT feature of each are
    // within a small distance of each other.
    bool areTheSame(SIFTFeature3D feature1, SIFTFeature3D feature2);


    // Checks to see if any of the currently active traces ar sufficiently
    // long to justify correlation of features.
    bool canFindLongTraces(unsigned frame_start);

    // Calculates the entire travelled distance of the trace, NOT just the
    // distance between the head and the tail.
    float traceLength(const Trace &trace);
    float traceLengthBetweenFrames(const Trace &trace, unsigned start_frame, unsigned end_frame);

    // Book-keeping for statistics.
    void updateLengthBins(const Trace &trace, const SIFTFeature3D &feature);

    // Checks to see if a trace has not been seen for a while, and thus is probably
    // no longer valid and should be removed.
    bool isTraceTooOld(const Trace &trace, unsigned cur_frame);

    bool isTraceCrooked(const Trace &trace, unsigned frame_start);

    void gatherTraceStatistics(void);
    float traceDistance(const Trace &trace1, const Trace &trace2, unsigned first_frame);

    std::vector<Trace> purgeInvalidTraces(const std::vector<Trace> &traces, unsigned first_frame);

    void addDummyArmTraces(std::vector<float> arm_joints, unsigned frame_num);
};

#endif

