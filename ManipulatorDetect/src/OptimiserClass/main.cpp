
#include <vector>
#include <cmath>
#include <iostream>

#include "Optimiser.h"
#include "HillClimbing.h"
#include "NelderMead.h"
#include "ParticleSwarm.h"
#include "SimulatedAnnealing.h"
#include "GeneticAlgorithm.h"

class SphereFunction : public OptimisableFunction {
  public:
    float eval(std::vector<float> &params){
        float result = 0.0f;
        for(unsigned i = 0; i < params.size(); i++){
            result += params.at(i) * params.at(i);
        }
        return result;
    }
};

class RosenbrockFunction : public OptimisableFunction {
  public:
    float eval(std::vector<float> &params){
        float result = 0.0f;
        for(unsigned i = 0; i < params.size()-1; i++){
            float xi = 10.0f * params.at(i), xi1 = 10.0f * params.at(i+1);
            result += (1.0f - xi)*(1.0f - xi) + 100.0f*(xi1 - xi*xi)*(xi1 - xi*xi);
        }
        return result;
    }
};

class RastriginFunction : public OptimisableFunction {
  public:
    float eval(std::vector<float> &params){
        float result = 10.0f * params.size();
        for(unsigned i = 0; i < params.size(); i++){
            float x = 10.0f * params.at(i);
            result += x*x - 10.0f*cos(2.0f*M_PI*x);
        }
        return result;
    }
};



int main(int argc, char **argv){
    //Optimiser *op = new ParticleSwarm(2000);
    //Optimiser *op = new NelderMead();
    //Optimiser *op = new SimulatedAnnealing();
    Optimiser *op = new GeneticAlgorithm(1000);

    OptimisableFunction *eval_func = new RosenbrockFunction();

    unsigned num_params = 100;
    std::vector<float> dim_magnitude(num_params, 0.5f);

    std::vector<float> min_vec = op->optimise(eval_func, num_params, 1000, dim_magnitude);

    std::cout << "Min Val: " << eval_func->eval(min_vec) << std::endl;
    std::cout << "Min Vec:";
    for(unsigned i = 0; i < min_vec.size(); i++){
        std::cout << " " << min_vec.at(i);
    }
    std::cout << std::endl;

}

