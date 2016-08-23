/*
 * SceneBestFit.h
 *
 *  Created on: 07/01/2010
 *      Author: osushkov
 */

#ifndef SCENEBESTFIT_H_
#define SCENEBESTFIT_H_

#include "Util/Transform.h"
#include "OptimiserClass/Optimiser.h"
#include "Reconstruction/ReconstructionFrame.h"

#include <vector>

class SceneBestFitFunction : public OptimisableFunction {
  public:
    SceneBestFitFunction(const std::vector<BestFitFeaturePair> &matching_features);
    float eval(std::vector<float> &params, float iter_fraction);

  private:
    const std::vector<BestFitFeaturePair> &matching_features;

    Vector3D applyTransform(const Transform &t, const Vector3D &vec);
};


class SceneBestFit {
  public:
    SceneBestFit();
    ~SceneBestFit();

    void setHint(Transform transform){
        hint_transform = transform;
    }

    float calculateBestFit(const std::vector<BestFitFeaturePair> &matching_features,
                           Transform &result, bool use_prev);

    void reset(void);

  private:
    Transform hint_transform;
    std::vector<float> prev;
    bool use_local;

};

#endif /* SCENEBESTFIT_H_ */
