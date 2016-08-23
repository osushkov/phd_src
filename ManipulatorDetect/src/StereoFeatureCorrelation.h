
#ifndef _StereoFeatureCorrelation_H_
#define _StereoFeatureCorrelation_H_

#include "Util/Common.h"
#include "SIFT/imgfeatures.h"
#include <vector>


struct StereoFeature {
    feature feature_left, feature_right;
    Vector3D position;
};

namespace StereoFeatureCorrelation {
    
    std::vector<StereoFeature> correlateFeatures(std::vector<feature> left, 
                                                 std::vector<feature> right);

};

#endif
