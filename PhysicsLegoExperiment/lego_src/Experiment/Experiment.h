
#ifndef _Experiment_H_
#define _Experiment_H_

#include <vector>
#include <map>
#include <iostream>
#include "ExperimentResult.h"
#include "../Object/ObjectLegoModel.h"
#include "../Visualisation/RenderObject.h"
#include "../Visualisation/WheeledBoxRenderObject.h"

enum ExperimentType {
    LIGHT_EXPERIMENT,
    NUM_EXPERIMENT_TYPES
};

class Experiment {
  public:
    Experiment(){};
    virtual ~Experiment(){};

    virtual ExperimentType getType(void) const = 0;

    // Returns P(Pose|Object) for the pose-object pair for this experiment.
    virtual std::vector<float>
    getConditionalResultProbabilities(unsigned num_possible_results, ObjectLegoModel model) = 0;

    virtual ExperimentResult performExperiment(void) = 0;
    virtual ExperimentResult performVirtualExperiment(ObjectLegoModel model, bool render) = 0;

    virtual bool canPerformExperiment(Transform arm_to_obj) = 0;

    virtual void print(void) = 0;

    virtual void save(std::ostream &out_stream) = 0;
    virtual void load(std::istream &in_stream) = 0;

  protected:
    std::map<unsigned, std::vector<float> > cached_object_conditional_probabilities;
};



#endif
