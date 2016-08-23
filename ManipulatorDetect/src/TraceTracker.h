
#ifndef _TraceTracker_H_
#define _TraceTracker_H_

#include "StereoFeatureCorrelation.h"
#include <vector>
#include <map>

struct TraceNode {
    StereoFeature feature;
    unsigned frame; // in which frame was this feature seen?
};

struct Trace {
    std::vector<TraceNode> nodes; // nodes in order of oldest to newest
};

class TraceTracker {
  public:
    TraceTracker();
    ~TraceTracker();

    void submitStereoFeatures(std::vector<StereoFeature> features, unsigned cur_frame);
    std::vector<Trace> getActiveTraces(void);

    bool isNMDMaxima(unsigned &frame_start, std::vector<StereoFeature> &features_start,
                     unsigned &frame_end, std::vector<StereoFeature> &features_end);

    std::vector<StereoFeature> getFrameFeatures(unsigned frame_num);
    bool findClosestFeature(StereoFeature feature, std::vector<StereoFeature> &all_features, 
                            unsigned &index);

  private:

    unsigned latest_frame;
    std::vector<Trace> all_traces; // history of ALL traces.

    // a list of all traces which can still be appended to
    std::vector<unsigned> cur_active_traces_ref;  

    std::map<unsigned, std::vector<StereoFeature> > frame_features;
    std::vector<float> nmd_ratings;
    bool nmd_maxima;
    unsigned last_snapshot_frame; 


    void insertFeature(StereoFeature feature, unsigned cur_frame);
    
    bool areTheSame(StereoFeature &feature1, StereoFeature &feature2);

    float dist(Vector3D v1, Vector3D v2); // TODO implement a proper Vector3D class with this already defined

    float currentNMDRating(void);
    float traceNMDRating(Trace trace);

    std::vector<Trace> getPotentialTraces(void);
    unsigned findBestEndFrame(const std::vector<Trace> &traces);
    bool traceContainsFrame(const Trace &trace, unsigned frame);

    bool canFindLongTraces(unsigned frame_start, unsigned frame_end);
};

#endif

