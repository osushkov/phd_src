
#ifndef _TraceTracker_H_
#define _TraceTracker_H_

#include "../Features/StereoFeatureCorrelation.h"
#include <vector>
#include <map>

struct TraceNode {
    StereoFeature feature;
    unsigned frame; // in which frame was this feature seen?
    bool is_arm;
};

struct Trace {
    float tag;
    std::vector<TraceNode> nodes; // nodes in order of oldest to newest
};

class TraceTracker {
  public:
    TraceTracker();
    ~TraceTracker();

    void submitStereoFeatures(std::vector<StereoFeature> features,
                              std::vector<bool> is_arm_flags,
                              unsigned cur_frame);
    std::vector<Trace> getActiveTraces(void);

    bool shouldCorrelate(unsigned &frame_start, std::vector<StereoFeature> &features_start,
                         unsigned &frame_end, std::vector<StereoFeature> &features_end,
                         unsigned cur_frame);

    std::vector<StereoFeature> getFrameFeatures(unsigned frame_num);
    bool findClosestFeature(StereoFeature feature, std::vector<StereoFeature> &all_features,
                            unsigned &index);

    void printBins(void);

  private:

    unsigned last_snapshot_frame;
    unsigned latest_frame;
    std::vector<Trace> all_traces; // history of ALL traces.

    // a list of all traces which can still be appended to
    std::vector<unsigned> cur_active_traces_ref;
    std::vector<unsigned> cur_active_arm_traces_ref;

    std::map<unsigned, std::vector<StereoFeature> > frame_features;


    // Insert the given feature into the set of currently active traces, if
    // no suitable active traces are found to include this feature, a new
    // trace will be created.
    void insertFeature(StereoFeature feature, bool is_arm, unsigned cur_frame);

    // Checks whether two five stereo features are almost certainly the
    // same feature, ie: both left and right SIFT feature of each are
    // within a small distance of each other.
    bool areTheSame(StereoFeature feature1, StereoFeature feature2);


    // Checks to see if any of the currently active traces ar sufficiently
    // long to justify correlation of features.
    bool canFindLongTraces(unsigned frame_start);

    // Calculates the entire travelled distance of the trace, NOT just the
    // distance between the head and the tail.
    float traceLength(const Trace &trace);
    float traceLengthBetweenFrames(const Trace &trace, unsigned start_frame, unsigned end_frame);

    // Book-keeping for statistics.
    void updateLengthBins(const Trace &trace, const StereoFeature &feature);

    // Checks to see if a trace has not been seen for a while, and thus is probably
    // no longer valid and should be removed.
    bool isTraceTooOld(const Trace &trace, unsigned cur_frame);

    void gatherTraceStatistics(void);
    float traceDistance(const Trace &trace1, const Trace &trace2);

    bool isArmTrace(const Trace &trace);
    std::vector<Trace> purgeInvalidTraces(const std::vector<Trace> &traces);

    void purgeArmTraces(void);
};

#endif

