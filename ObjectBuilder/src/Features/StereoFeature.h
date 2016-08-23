/*
 * StereoFeature.h
 *
 *  Created on: 26/06/2009
 *      Author: osushkov
 */

#ifndef STEREOFEATURE_H_
#define STEREOFEATURE_H_

#include "../Util/Common.h"
#include "SIFT/imgfeatures.h"

struct StereoFeature {
    feature feature_left, feature_right;
    Vector3D position;
};


#endif /* STEREOFEATURE_H_ */
