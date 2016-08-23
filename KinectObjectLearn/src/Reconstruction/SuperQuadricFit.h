/*
 * SuperQuadricFit.h
 *
 *  Created on: 17/04/2010
 *      Author: osushkov
 */

#ifndef SUPERQUADRICFIT_H_
#define SUPERQUADRICFIT_H_

#include "../Util/Transform.h"

struct SuperQuadricFit {
    double e1, e2;
    double A, B, C;
    double xt, yt;
    Transform transform;
};


#endif /* SUPERQUADRICFIT_H_ */
