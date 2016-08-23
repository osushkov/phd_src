/*
 * SuperQuadricBestFit.h
 *
 *  Created on: 28/10/2009
 *      Author: osushkov
 */

#ifndef SUPERQUADRICBESTFIT_H_
#define SUPERQUADRICBESTFIT_H_

#include "ReconstructionManager.h"
#include "../OptimiserClass/Optimiser.h"
#include "SuperQuadricFit.h"
#include "SuperQuadric.h"


class SuperQuadricBestFitFunction : public OptimisableFunction {
  public:
    SuperQuadricBestFitFunction(std::vector<Vector3D> sift_points);

    float eval(std::vector<float> &params);

  private:
    std::vector<Vector3D> sift_points;

    float pointsError(SuperQuadricFit quadric, SuperQuadric &sq, const std::vector<Vector3D> &points);
};

class SuperQuadricBestFit {
  public:
    SuperQuadricBestFit();
    ~SuperQuadricBestFit();

    SuperQuadricFit calculateShapeHypotheses(std::vector<Vector3D> sift_points);

  private:

      std::vector<Vector3D> getPointsFit(SuperQuadricFit &sq_fit, std::vector<Vector3D> points, float max_error);

};


#endif /* SUPERQUADRICBESTFIT_H_ */
