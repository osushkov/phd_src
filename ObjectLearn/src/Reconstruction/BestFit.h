/*
 * BestFit.h
 *
 *  Created on: 07/07/2009
 *      Author: osushkov
 */

#ifndef BESTFIT_H_
#define BESTFIT_H_

#include "../Util/Transform.h"
#include "../OptimiserClass/Optimiser.h"
#include "ReconstructionFrame.h"

#include <vector>


class BestFitFunction : public OptimisableFunction {
  public:
    BestFitFunction(std::vector<BestFitFeaturePair> matching_features, Vector3D region_centre);
    float eval(std::vector<float> &params, float iter_frac);

  private:
    std::vector<BestFitFeaturePair> matching_features;
    Vector3D region_centre;
};


class BestFit {
  public:
    BestFit();
    ~BestFit();

    float calculateBestFit(const std::vector<BestFitFeaturePair> &matching_features,
                           Transform &result, bool use_prev);

    void setHintShift(Vector3D hint);

  private:
    Vector3D hint_shift;

    std::vector<BestFitFeaturePair> getPointsFit(Transform &t, std::vector<BestFitFeaturePair> all_pairs, float max_error);
};

#endif /* BESTFIT_H_ */
