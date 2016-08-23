
#ifndef _Experiment_H_
#define _Experiment_H_

#include <vector>
#include <map>
#include <iostream>
#include "ExperimentResult.h"
#include "../Object/ObjectPhysModel.h"
#include "../Physics/PhysicsWorld.h"
#include "../Physics/PhysicsObject.h"
#include "../Visualisation/RenderObject.h"

enum ExperimentType {
    DROP_EXPERIMENT,
    NUM_EXPERIMENT_TYPES
};

class Experiment {
  public:
    Experiment(){};
    virtual ~Experiment(){};

    virtual ExperimentType getType(void) const = 0;

    // Returns P(Pose|Object) for the pose-object pair for this experiment.
    virtual std::vector<float>
    getConditionalResultProbabilities(unsigned num_possible_results, ObjectPhysModel model) = 0;

    // Returns P(Pose) for every pose.
    virtual std::vector<float> 
    getResultProbabilities(unsigned num_possible_results, std::vector<ObjectPhysModel> all_models) = 0;

    virtual ExperimentResult performExperiment(void) = 0;
    virtual ExperimentResult performVirtualExperiment(PhysicsObject* phys_obj, RenderObject *render_obj=NULL) = 0;

    virtual bool canPerformExperiment(Transform arm_to_obj) = 0;

    virtual void setPhysicsWorld(PhysicsWorld *phys_world) = 0;
    virtual void print(void) = 0;

    virtual void save(std::ostream &out_stream) = 0;
    virtual void load(std::istream &in_stream) = 0;

  protected:
    std::vector<float> cached_result_probabilities;
    std::map<unsigned, std::vector<float> > cached_object_conditional_probabilities;
};



#endif
