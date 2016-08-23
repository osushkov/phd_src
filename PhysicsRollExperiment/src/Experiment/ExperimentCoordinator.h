
#ifndef _ExperimentCoordinator_H_
#define _ExperimentCoordinator_H_

#include "Experiment.h"
#include "DropExperiment.h"
#include "../Object/Object.h"
#include "../Object/ObjectPhysModel.h"
#include "../Physics/PhysicsWorld.h"

#include <vector>


class ExperimentCoordinator {
  public:
    static ExperimentCoordinator& instance(void);

    void initialise(std::vector<ObjectPhysModel> phys_models,
                    std::vector<Experiment*> experiments,
                    float ramp_theta);

    void run(Object *obj);

  private:
    bool is_initialised;
    Vector3D ramp_normal;

    std::vector<Experiment*> all_experiments;
    std::vector<ObjectPhysModel> all_phys_models;
    std::vector<float> cur_model_pd; // current probability distribution.
    PhysicsWorld* phys_world;

    ObjectPhysModel *real_model;

    void initialiseRamp(float ramp_theta, float ramp_xend);
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
