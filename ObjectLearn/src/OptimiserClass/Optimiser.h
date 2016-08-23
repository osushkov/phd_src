
#ifndef _Optimiser_H_
#define _Optimiser_H_

#include <vector>

class OptimisableFunction {
  public:
    virtual ~OptimisableFunction(){}
    virtual float eval(std::vector<float> &params, float iter_fraction) = 0;
};


class Optimiser {
  public:
    virtual ~Optimiser(){}
    virtual std::vector<float> optimise(OptimisableFunction *target, unsigned num_params,
                                        unsigned max_evals, float target_fitness=0.0f) = 0;

};

#endif

