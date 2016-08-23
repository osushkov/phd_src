
#ifndef _Optimiser_H_
#define _Optimiser_H_

#include <vector>

class OptimisableFunction {
  public:
    OptimisableFunction() : debug(false) {}
    
    virtual ~OptimisableFunction(){}
    virtual float eval(std::vector<float> &params) = 0;

    void setDebug(bool var){ debug = var; }

  protected:
    bool debug;
};


class Optimiser {
  public:
    virtual ~Optimiser(){}
    virtual std::vector<float> optimise(OptimisableFunction *target, unsigned num_params,
                                        unsigned max_evals, float target_fitness=0.0f) = 0;

};

#endif

