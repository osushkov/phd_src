
#ifndef _StereoFeatureCorrelation_H_
#define _StereoFeatureCorrelation_H_

#include "../Util/Common.h"
#include "SIFT/imgfeatures.h"
#include "StereoFeature.h"
#include "../Visualisation/SceneRenderer.h"
#include <vector>


namespace StereoFeatureCorrelation {

    std::vector<StereoFeature> correlateFeatures(std::vector<feature> left,
                                                 std::vector<feature> right,
                                                 unsigned res_height);

    // First entry: interest point scene features.
    // Second entry: vertical edge based scene features.
    std::pair<std::vector<SceneFeature>,std::vector<SceneFeature> >
    findAllSceneFeatures(IplImage *img_left, IplImage *img_right,
                         const std::vector<StereoFeature> &stereo_features);

};

#endif
