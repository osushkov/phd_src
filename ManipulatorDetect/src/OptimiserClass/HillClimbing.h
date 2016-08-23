
#ifndef _HillClimbing_H_
#define _HillClimbing_H_

#include "Optimiser.h"

class HillClimbing : public Optimiser {
  public:
    HillClimbing();
    ~HillClimbing();

     std::vector<float> optimise(OptimisableFunction *target, unsigned num_params, 
                                 unsigned max_evals, std::vector<float> dimension_magnitude);

  private:
    std::vector<float> current_min_vec;
    float current_min_val;

    std::vector<float> genRandomOffset(std::vector<float> &dimension_magnitude);
    std::vector<float> addVectors(std::vector<float> &vec1, std::vector<float> &vec2);
    void clipVector(std::vector<float> &vec, float min, float max);
};

#endif
