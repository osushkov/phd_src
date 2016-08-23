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
    BestFitFunction(std::vector<BestFitFeaturePair> matching_featurese);
    float eval(std::vector<float> &params);

  private:
    std::vector<BestFitFeaturePair> matching_features;
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

};

#endif /* BESTFIT_H_ */
