
#ifndef _CorrelatedFrameParser_H_
#define _CorrelatedFrameParser_H_

#include <vector>
#include <fstream>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "../Features/StereoFeatureCorrelation.h"
#include "../Features/FeatureMemory/FeatureMemory.h"


struct CorrelatedFrame {
    std::string left_frame_filename;
    std::string right_frame_filename;
    std::string features_filename;

    std::vector<StereoFeature> features;
    std::vector<FeatureObjectType> feature_types;

    std::string object_type;

    bool fully_loaded;
};


class CorrelatedFrameParser {
  public:
    CorrelatedFrameParser(std::string path);
    ~CorrelatedFrameParser();

    // This write a new entry into the correlated frames table, writes out the left
    // and right frames as separate files, and the feature into a separate data file.
    void write(IplImage *left_frame, IplImage *right_frame,
               const std::vector<StereoFeature> &features,
               const std::vector<FeatureObjectType> &feature_types,
               std::string object_type);

    std::vector<CorrelatedFrame> read(void);
    void loadCorrelatedFrame(CorrelatedFrame &frame);

  private:

    std::string path;
    std::fstream root_file;
    unsigned current_frame_id;
    unsigned cur_output_num;

    void writeOutFeatures(std::ofstream &out_file,
                          const std::vector<StereoFeature> &features,
                          const std::vector<FeatureObjectType> &feature_types);

    void readInFeatures(std::ifstream &in_file,
                        std::vector<StereoFeature> &features,
                        std::vector<FeatureObjectType> &feature_types);

    std::string pathToRootFile(std::string path);

};

#endif
