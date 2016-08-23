
#ifndef _HillClimbing_H_
#define _HillClimbing_H_

#include "Optimiser.h"

class HillClimbing : public Optimiser {
  public:
    HillClimbing();
    ~HillClimbing();

     std::vector<float> optimise(OptimisableFunction *target, unsigned num_params,
                                 unsigned max_evals, float target_fitness=0.0f);

  private:
    std::vector<float> current_min_vec;
    float current_min_val;

    std::vector<float> genRandomOffset(unsigned dim);
    std::vector<float> addVectors(std::vector<float> &vec1, std::vector<float> &vec2);
    void clipVector(std::vector<float> &vec, float min, float max);
};

#endif

