
#ifndef _GeneticAlgorithm_H_
#define _GeneticAlgorithm_H_

#include "Optimiser.h"
#include <vector>


class GeneticAlgorithm : public Optimiser {
  public:
    GeneticAlgorithm(unsigned population_size);
    ~GeneticAlgorithm();

    std::vector<float> optimise(OptimisableFunction *target, unsigned num_params,
                                unsigned max_evals, float target_fitness=0.0f);

  private:

    struct GAIndividual {
        std::vector<float> genome;
        float fitness;
    };

    struct GAComp {
        bool operator()(const GAIndividual &var1, const GAIndividual &var2){
            return var1.fitness < var2.fitness;
        }
    };

    unsigned num_parameters;
    unsigned population_size;
    std::vector<GAIndividual> population;


    std::vector<GAIndividual> generateOffspring(std::vector<GAIndividual> &parent_pool, unsigned num_offspring,
                                                OptimisableFunction *target);
    GAIndividual createNewIndividual(OptimisableFunction *target);
    GAIndividual cross(GAIndividual &var1, GAIndividual &var2, OptimisableFunction *target);
    void mutate(GAIndividual &var);
};

#endif

