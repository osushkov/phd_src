
#include "HillClimbing.h"
#include "Common.h"
#include <cstdlib>
#include <cassert>
#include <iostream>

HillClimbing::HillClimbing(){
}

HillClimbing::~HillClimbing(){
}

std::vector<float> HillClimbing::optimise(OptimisableFunction *target, unsigned num_params,
                                          unsigned max_evals,  float target_fitness){

    float start_step_size = 0.1f;
    float step_size = start_step_size;
    for(unsigned i = 0; i < num_params; i++){
        float param = rand_range(0.0f, 1.0f); // generate a number between 0.0 and 1.0
        current_min_vec.push_back(param);
    }

    current_min_val = target->eval(current_min_vec);

    bool gained_new_minimum = false;
    std::vector<float> offset;
    for(unsigned iter = 0; iter < max_evals; iter++){
        // If on the previous iteration we found a new minimum we reuse
        // the same offset vector, it might be a pretty good direction.
        if(!gained_new_minimum){
            offset = genRandomOffset(num_params);
            for(unsigned i = 0; i < offset.size(); i++){
                offset[i] *= step_size;
            }
        }

        std::vector<float> new_vec = addVectors(current_min_vec, offset);
        clipVector(new_vec, 0.0f, 1.0f);

        float val = target->eval(new_vec);
        if(val < current_min_val){
            current_min_val = val;
            current_min_vec = new_vec;
            gained_new_minimum = true;
            std::cout << "Got new min: " << val << " on iter: " << iter << std::endl;

            if(current_min_val <= target_fitness){
                break;
            }
        }
        else{
            gained_new_minimum = false;
        }

        step_size = start_step_size * (1.0f - (float)iter/(float)max_evals);
    }

    return current_min_vec;
}

std::vector<float> HillClimbing::genRandomOffset(unsigned dim){
    std::vector<float> result;
    for(unsigned i = 0; i < dim; i++){
        float param = rand_range(0.0f, 1.0f);
        result.push_back(param);
    }
    return result;
}

std::vector<float> HillClimbing::addVectors(std::vector<float> &vec1,
                                            std::vector<float> &vec2){
    assert(vec1.size() == vec2.size());

    std::vector<float> result;
    for(unsigned i = 0; i < vec1.size(); i++){
        result.push_back(vec1.at(i) + vec2.at(i));
    }
    return result;
}

void HillClimbing::clipVector(std::vector<float> &vec, float min, float max){
    for(unsigned i = 0; i < vec.size(); i++){
        if(vec.at(i) < min){ vec.at(i) = min; }
        else if(vec.at(i) > max){ vec.at(i) = max; }
    }
}

