/*
 * MemoryEvaluator.h
 *
 *  Created on: 23/04/2009
 *      Author: osushkov
 */

#ifndef MEMORYEVALUATOR_H_
#define MEMORYEVALUATOR_H_

#include "../Features/StereoFeature.h"
#include "../Features/FeatureMemory/ObjectSnapshotDB.h"

#include <vector>
#include <string>

#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>


class MemoryEvaluator {
  public:
    MemoryEvaluator(std::string root_file, std::string path_prefix);
    ~MemoryEvaluator();

    void evaluate(ObjectSnapshotDB &snapshot_db,
                  unsigned &total_features,
                  unsigned &correct,
                  unsigned &incorrect);

    void loadBackgroundFeatures(std::string path, std::string root_file);
    void addObjectFeatures(std::vector<StereoFeature> &new_object_features);
    bool isObjectFeature(StereoFeature sfeature);

    void evalManualSegmentation(std::string path, std::string root_file);

  private:

    std::vector<std::pair<IplImage*,IplImage*> > images;
    std::vector<std::pair<IplImage*,IplImage*> > masks;
    std::vector<std::vector<StereoFeature> > frame_features;

    std::vector<feature> background_features;
    std::vector<StereoFeature> object_features;

    bool isObjectPixel(IplImage *mask, int xcoord, int ycoord);
    void filterFeaturesOnScale(std::vector<StereoFeature> &features, float max_scale);
};

#endif /* MEMORYEVALUATOR_H_ */
