
#ifndef _CorrelatedFrameLearner_H_
#define _CorrelatedFrameLearner_H_

#include "CorrelatedFrameParser.h"

#include <string>
#include <map>

class CorrelatedFrameLearner {
  public:

    CorrelatedFrameLearner();
    ~CorrelatedFrameLearner();

    void loadCorrelatedFrames(std::string filename);
    void processCorrelatedFrames(std::string object_name);
    void writeOutCorrelatedFrames(std::string filename);

  private:

    CorrelatedFrameParser *parser;
    std::map<std::string, std::vector<CorrelatedFrame> > object_correlated_frames;
    unsigned num_corrections;


    void doCorrelation(std::vector<CorrelatedFrame> &object_frames,
                       std::vector<CorrelatedFrame> &reference_frames,
                       std::string object_name);

    bool isSignificantCorrelation(CorrelatedFrame &object_frame, CorrelatedFrame &reference_frame);
    void doFrameCorrelation(CorrelatedFrame &object_frame, CorrelatedFrame &reference_frame);

    void renderCorrelatedFrame(CorrelatedFrame &frame, std::string window_name);
};

#endif

