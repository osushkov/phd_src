
#include "ExperimentResultClassifier.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <list>

ExperimentResultClassifier& ExperimentResultClassifier::instance(void){
    static ExperimentResultClassifier result;
    return result;
}

ExperimentResultClassifier::ExperimentResultClassifier() : is_built(false) {

}

ExperimentResultClassifier::~ExperimentResultClassifier(){

}

void ExperimentResultClassifier::buildClassifier(){
    std::cout << "Building Experiment Result Classifier" << std::endl;
    prototype_results.clear();

    const float min_x = -25.0f;
    const float max_x = 25.0f;
    const float min_y = -25.0f;
    const float max_y = 25.0f;

    const float pos_step = 5.0f;
    const float angle_step = 10.0f*M_PI/180.0f;

    for(float x = min_x; x <= max_x; x += pos_step){
        for(float y = min_y; y <= max_y; y += pos_step){
            for(float angle = -M_PI; angle < M_PI; angle += angle_step){
                ExperimentResult new_prototype(angle, Vector2D(x, y), prototype_results.size());
                prototype_results.push_back(new_prototype);
            }
        }
    }

    is_built = true;
}

int ExperimentResultClassifier::classifyExperimentResult(ExperimentResult var){
    float closest_dist = 0.0f;
    int best_label = -1;

    for(unsigned i = 0; i < prototype_results.size(); i++){
        float dist = prototype_results[i].distance(var);
        if(i == 0 || dist < closest_dist){
            closest_dist = dist;
            best_label = prototype_results[i].getLabel();
        }
    }

    return best_label;
}

ExperimentResult ExperimentResultClassifier::getPrototypeResult(unsigned label){
    if(label >= prototype_results.size()){
        std::cerr << "incorrect prototype result label: " << label << std::endl;
    }

    return prototype_results.at(label);
}


bool ExperimentResultClassifier::isBuilt(void) const {
    return is_built;
}

unsigned ExperimentResultClassifier::getNumLabels(void) const {
    return prototype_results.size();
}
