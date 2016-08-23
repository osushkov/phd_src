
#include "GeneticAlgorithm.h"
#include "Common.h"

#include <vector>
#include <cassert>
#include <algorithm>
#include <iostream>

#define MUTATION_RATE 0.1f
#define MUTATION_AMPLITUDE 0.1f

GeneticAlgorithm::GeneticAlgorithm(unsigned population_size) : population_size(population_size) {
}

GeneticAlgorithm::~GeneticAlgorithm(){
}


std::vector<float> GeneticAlgorithm::optimise(OptimisableFunction *target, unsigned num_params, 
                            unsigned max_evals, std::vector<float> dimension_magnitude){

    num_parameters = num_params;

    for(unsigned i = 0; i < population_size; i++){
        population.push_back(createNewIndividual(target));
    }
    assert(population.front().genome.size() == num_params);

    GAComp comp;
    sort(population.begin(), population.end(), comp);
    assert(population.front().fitness <= population.back().fitness);

    float best_fitness = 0.0;
    float prev_best = population.front().fitness;

    for(unsigned iter = 0; iter < max_evals; iter++){
        std::vector<GAIndividual> new_generation = generateOffspring(population, (unsigned)(population.size()*0.8f), target);
        
        unsigned num_oldies = population.size() - new_generation.size();
        for(unsigned i = 0; i < num_oldies; i++){
            new_generation.push_back(population.at(i));
        }
        assert(new_generation.size() == population_size);

        population = new_generation;
        sort(population.begin(), population.end(), comp);
        assert(population.front().fitness <= population.back().fitness);
        assert(population.at(0).fitness <= population.at(1).fitness);
        assert(population.at(0).fitness <= prev_best);
        prev_best = population.at(0).fitness;

        if(population.front().fitness < best_fitness || iter == 0){
            best_fitness = population.front().fitness;
            std::cout << iter << " : " << best_fitness << std::endl;
        }
    }

    return population.at(0).genome;
}


std::vector<GeneticAlgorithm::GAIndividual> GeneticAlgorithm::generateOffspring(std::vector<GAIndividual> &parent_pool, unsigned num_offspring,
                                                              OptimisableFunction *target){
    assert(parent_pool.size() >= 2); // parent pool needs to be at least of size 2, otherwise nobody to cross with.

    std::vector<GAIndividual> result;

    for(unsigned i = 0; i < num_offspring; i++){
        unsigned parent1_index = (unsigned)abs((int)gaussian_noise(0.0f, parent_pool.size()/8.0f));

        unsigned parent2_index;
        while((parent2_index = (unsigned)abs((int)gaussian_noise(0.0f, parent_pool.size()/8.0f))) == parent1_index){}

        if(parent1_index >= parent_pool.size()){ parent1_index = 0; }
        if(parent2_index >= parent_pool.size()){ parent2_index = 1; }

        GAIndividual new_individual = cross(parent_pool.at(parent1_index), parent_pool.at(parent2_index), target);
        mutate(new_individual);

        new_individual.fitness = target->eval(new_individual.genome);
        result.push_back(new_individual);
    }

    return result;
}


GeneticAlgorithm::GAIndividual GeneticAlgorithm::createNewIndividual(OptimisableFunction *target){
    assert(target != NULL);

    GAIndividual result;
    for(unsigned i = 0; i < num_parameters; i++){
        result.genome.push_back(rand_range(-1.0f, 1.0f));
    }

    result.fitness = target->eval(result.genome);
    return result;
}

GeneticAlgorithm::GAIndividual GeneticAlgorithm::cross(GAIndividual &var1, GAIndividual &var2, OptimisableFunction *target){
    assert(var1.genome.size() == var2.genome.size());
    assert(target != NULL);

    GAIndividual result;
    for(unsigned i = 0; i < var1.genome.size(); i++){
        //float p = rand_range(0.0f, 1.0f);
        //result.genome.push_back(p*var1.genome.at(i) + (1.0f - p)*var2.genome.at(i));
        
        
        if(rand_range(0.0f, 1.0f) < 0.5f){
            result.genome.push_back(var1.genome.at(i));
        }
        else{
            result.genome.push_back(var2.genome.at(i));
        }
    }

    return result;
}

void GeneticAlgorithm::mutate(GAIndividual &var){
    for(unsigned i = 0; i < var.genome.size(); i++){
        if(rand_range(0.0f, 1.0f) < MUTATION_RATE){
            var.genome.at(i) += rand_range(-MUTATION_AMPLITUDE, MUTATION_AMPLITUDE);
            var.genome.at(i) = clip(var.genome.at(i), -1.0f, 1.0f);
        }
    }
}

