
#include "SimulatedAnnealing.h"
#include "Common.h"
#include <cstdlib>
#include <cassert>
#include <iostream>

SimulatedAnnealing::SimulatedAnnealing(){
}

SimulatedAnnealing::~SimulatedAnnealing(){
}

std::vector<float> SimulatedAnnealing::optimise(OptimisableFunction *target, unsigned num_params, 
                                          unsigned max_evals, std::vector<float> dimension_magnitude){
    assert(dimension_magnitude.size() == num_params);

    std::vector<float> reduction_vector;
    for(unsigned i = 0; i < num_params; i++){
        float param = rand_range(-1.0f, 1.0f); // generate a number between -1.0 and 1.0
        current_min_vec.push_back(param);

        reduction_vector.push_back(-dimension_magnitude.at(i)/(float)max_evals);
    }

    current_min_val = target->eval(current_min_vec);

    bool gained_new_minimum = false;
    std::vector<float> offset;
    for(unsigned iter = 0; iter < max_evals; iter++){
        // If on the previous iteration we found a new minimum we reuse
        // the same offset vector, it might be a pretty good direction.
        if(!gained_new_minimum){
            offset = genRandomOffset(dimension_magnitude);
        }

        std::vector<float> new_vec = addVectors(current_min_vec, offset);
        clipVector(new_vec, -1.0f, 1.0f);

        float val = target->eval(new_vec);
        if(acceptNewPosition(val)){
            current_min_val = val;
            current_min_vec = new_vec;
            gained_new_minimum = true;
            std::cout << "Got new min: " << val << " on iter: " << iter << std::endl;
        }
        else{
            gained_new_minimum = false;
        }

        dimension_magnitude = addVectors(dimension_magnitude, reduction_vector);
    }

    return current_min_vec;
}

bool SimulatedAnnealing::acceptNewPosition(float new_value){
    return new_value < current_min_val;
}

std::vector<float> SimulatedAnnealing::genRandomOffset(std::vector<float> &dimension_magnitude){
    std::vector<float> result;
    for(unsigned i = 0; i < dimension_magnitude.size(); i++){
        float param = rand_range(-1.0f, 1.0f) * dimension_magnitude.at(i);
        result.push_back(param);
    }
    return result;
}

std::vector<float> SimulatedAnnealing::addVectors(std::vector<float> &vec1, std::vector<float> &vec2){
    assert(vec1.size() == vec2.size());

    std::vector<float> result;
    for(unsigned i = 0; i < vec1.size(); i++){
        result.push_back(vec1.at(i) + vec2.at(i));
    }
    return result;
}

void SimulatedAnnealing::clipVector(std::vector<float> &vec, float min, float max){
    for(unsigned i = 0; i < vec.size(); i++){
        if(vec.at(i) < min){ vec.at(i) = min; }
        else if(vec.at(i) > max){ vec.at(i) = max; }
    }
}

