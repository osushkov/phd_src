/*
 * RRNelderMead.h
 *
 *  Created on: 25/03/2009
 *      Author: osushkov
 */

#ifndef RRNELDERMEAD_H_
#define RRNELDERMEAD_H_

#include "Optimiser.h"
#include <vector>

class RRNelderMead : public Optimiser {
  public:
    RRNelderMead(unsigned num_workers);
    ~RRNelderMead();

    std::vector<float> optimise(OptimisableFunction *target, unsigned num_params,
                                unsigned max_evals, float target_fitness=0.0f);

  private:
    unsigned num_workers;

};

#endif /* RRNELDERMEAD_H_ */
