
#ifndef _DepthFeaturePublisher_H_
#define _DepthFeaturePublisher_H_

#include <vector>
#include "StereoFeatureCorrelation.h"

namespace DepthFeaturePublisher {

    void initialise(void);
    void submitNewFeatures(const std::vector<StereoFeature> &new_features);


};

#endif

