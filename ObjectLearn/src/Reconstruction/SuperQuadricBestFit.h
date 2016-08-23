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
#include "../SilhouetteMatch/ObjectShapeVerifier.h"
#include "SuperQuadricFit.h"


class SuperQuadricBestFitFunction : public OptimisableFunction {
  public:
    SuperQuadricBestFitFunction(std::vector<Vector3D> sift_points,
                                std::vector<Vector3D> feature_points,
                                std::vector<Vector3D> edge_points,
                                ObjectShapeVerifier &shape_verifier,
                                bool use_silhouette);

    float eval(std::vector<float> &params, float iter_fraction);

  private:
    std::vector<Vector3D> sift_points;
    std::vector<Vector3D> feature_points;
    std::vector<Vector3D> edge_points;
    ObjectShapeVerifier &shape_verifier;
    bool use_silhouette;

    float pointsError(SuperQuadricFit quadric, SuperQuadric &sq, const std::vector<Vector3D> &points);
};

class SuperQuadricBestFit {
  public:
    SuperQuadricBestFit(ObjectShapeVerifier &shape_verifier);
    ~SuperQuadricBestFit();

    std::vector<std::pair<SuperQuadricFit,float> >
    calculateShapeHypotheses(std::vector<Vector3D> sift_points,
                             std::vector<Vector3D> feature_points,
                             std::vector<Vector3D> edge_points);

  private:
    ObjectShapeVerifier &shape_verifier;

    std::vector<Vector3D> getPointsFit(SuperQuadricFit &sq_fit, std::vector<Vector3D> points, float max_error);

};


#endif /* SUPERQUADRICBESTFIT_H_ */
