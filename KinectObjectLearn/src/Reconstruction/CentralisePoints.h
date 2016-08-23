/*
 * CentralisePoints.h
 *
 *  Created on: 29/10/2009
 *      Author: osushkov
 */

#ifndef CENTRALISEPOINTS_H_
#define CENTRALISEPOINTS_H_

#include "ReconstructionManager.h"
#include "../OptimiserClass/Optimiser.h"

class CentralisePointsFitFunction : public OptimisableFunction {
  public:
    CentralisePointsFitFunction(const std::vector<Vector3D> &points);

    float eval(std::vector<float> &params);

  private:
    const std::vector<Vector3D> &points;

    Vector3D applyTransform(const Transform &t, const Vector3D &vec);

};

class CentralisePoints {
  public:
    CentralisePoints();
    ~CentralisePoints();

    float calculateTransform(const std::vector<Vector3D> &points, Transform &result);

  private:

};


#endif /* CENTRALISEPOINTS_H_ */
