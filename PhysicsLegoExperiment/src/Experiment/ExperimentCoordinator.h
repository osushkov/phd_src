
#ifndef _ExperimentCoordinator_H_
#define _ExperimentCoordinator_H_

#include "Experiment.h"
#include "LightExperiment.h"
#include "../Object/Object.h"
#include "../Object/ObjectLegoModel.h"

#include <vector>


class ExperimentCoordinator {
  public:
    static ExperimentCoordinator& instance(void);

    void initialise(std::vector<ObjectLegoModel> phys_models,
                    std::vector<Experiment*> experiments);

    void run(Object *obj);

  private:
    bool is_initialised;
    Vector3D ramp_normal;

    std::vector<Experiment*> all_experiments;
    std::vector<ObjectLegoModel> all_phys_models;
    std::vector<float> cur_model_pd; // current probability distribution.

    ObjectLegoModel real_model;

    void initialiseRealModel(void);

    void performWorldExperiment(Object *obj, std::vector<std::pair<float,Experiment*> > experiment_entropies);
    float calculateExpectedKLDivergence(Experiment* experiment, std::vector<float> &prior);
    bool isValidDistribution(const std::vector<float> &distribution);
    void updateModelPriors(ExperimentResult experiment_result, Experiment *experiment);

    ExperimentCoordinator();
    ~ExperimentCoordinator();

    friend class ExperimentParallelWorker;
};

#endif
