/*
 * SIFTFeature3D.h
 *
 *  Created on: 26/06/2009
 *      Author: osushkov
 */

#ifndef SIFTFEATURE3D_H_
#define SIFTFEATURE3D_H_

#include "../Util/Vector3D.h"
#include "SIFT/imgfeatures.h"

struct SIFTFeature3D {
    feature sift_feature;
    Vector3D position;
};

#endif /* SIFTFEATURE3D_H_ */
