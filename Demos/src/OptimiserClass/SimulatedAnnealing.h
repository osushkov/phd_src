
#ifndef _SimulatedAnnealing_H_
#define _SimulatedAnnealing_H_

#include "Optimiser.h"
#include <vector>

class SimulatedAnnealing : public Optimiser {
  public:
    SimulatedAnnealing();
    ~SimulatedAnnealing();

    std::vector<float> optimise(OptimisableFunction *target, unsigned num_params,
                                unsigned max_evals, float target_fitness=0.0f);

  private:
    std::vector<float> current_min_vec;
    float current_min_val;

    float temperature;

    bool acceptNewPosition(float new_value);
    std::vector<float> genRandomOffset(unsigned dim);
    std::vector<float> addVectors(std::vector<float> &vec1, std::vector<float> &vec2);
    void clipVector(std::vector<float> &vec, float min, float max);

};

#endif

