
#include "ExperimentCoordinator.h"
#include "ExperimentResultClassifier.h"
#include "../Control.h"
#include "../Arm/Arm.h"
#include "../Util/Common.h"
#include "../Util/Geometry.h"
#include "../Util/Timer.h"
#include "../Util/ParallelServer.h"
#include "../Visualisation/SceneRenderer.h"
#include "../Visualisation/PlaneRenderObject.h"
#include <cstdio>
#include <fstream>
#include <iostream>

#define _USE_MATH_DEFINES
#include <math.h>

static std::ofstream model_pd_file = std::ofstream("data/model_pd.dat");
static bool is_loaded = false;

static void outputModelPD(std::vector<float> model_pd){

    for(unsigned i = 0; i < model_pd.size(); i++){
        model_pd_file << model_pd[i] << " ";
    }

    model_pd_file << std::endl;
}

ExperimentCoordinator& ExperimentCoordinator::instance(void){
    static ExperimentCoordinator coordinator;
    return coordinator;
}

ExperimentCoordinator::ExperimentCoordinator() :
        is_initialised(false) {

}

ExperimentCoordinator::~ExperimentCoordinator(){
	for(unsigned i = 0; i < all_experiments.size(); i++){
		delete all_experiments[i];
	}
}

void ExperimentCoordinator::initialise(std::vector<ObjectLegoModel> phys_models, 
                                       std::vector<Experiment*> experiments){
    if(is_initialised){
        return;
    }

    if(!ExperimentResultClassifier::instance().isBuilt()){
        ExperimentResultClassifier::instance().buildClassifier();
    }

    all_experiments = experiments;

    all_phys_models = phys_models;
    for(unsigned i = 0; i < all_phys_models.size(); i++){
        cur_model_pd.push_back(1.0f/all_phys_models.size());
    }
    
    Common::formProbabilityDistribution(cur_model_pd);

    initialiseRealModel();

    is_initialised = true;
}

struct EntropyComp {
    EntropyComp(float cur_entropy) : cur_entropy(cur_entropy) {}

    bool operator()(const std::pair<float,Experiment*> &a, const std::pair<float,Experiment*> &b){
        return a.first > b.first;
    }

    const float cur_entropy;
};

void ExperimentCoordinator::run(Object *obj){
    float cur_entropy = Common::distributionEntropy(cur_model_pd);

    outputModelPD(cur_model_pd);

    Util::Timer timer;
    timer.start();
    for(unsigned experiment_iter = 0; experiment_iter < 6; experiment_iter++){
        Common::printVector(cur_model_pd);

        std::vector<std::pair<float,Experiment*> > experiment_entropies;
        for(unsigned i = 0; i < all_experiments.size(); i++){
            float exp_info = calculateExpectedKLDivergence(all_experiments[i], cur_model_pd);
            experiment_entropies.push_back(std::pair<float,Experiment*>(exp_info, all_experiments[i]));
        }

        EntropyComp comp(cur_entropy);
        std::sort(experiment_entropies.begin(), experiment_entropies.end(), comp);
        assert(experiment_entropies.front().first <= experiment_entropies.back().first);

        std::cout << "done" << std::endl;
        //getchar();
        performWorldExperiment(obj, experiment_entropies);
        cur_entropy = Common::distributionEntropy(cur_model_pd);

        outputModelPD(cur_model_pd);
    }
    timer.stop();
}

void ExperimentCoordinator::initialiseRealModel(void){
    // This is where for testing I create the actual model it is trying to find.
    //real_model = ObjectLegoModel(0, 1, 0, -1);
    
    real_model = ObjectLegoModel(-1, -1, -1, -1, 
        std::pair<Vector3D,Vector3D>(Vector3D(5.5f, 5.5f, 0.0f), Vector3D(0.707f, 0.0f, -0.707)),
        std::pair<Vector3D,Vector3D>(Vector3D(5.5f, -5.5f, 0.0f), Vector3D(0.707f, 0.0f, -0.707)));
}

void ExperimentCoordinator::performWorldExperiment(Object *obj, std::vector<std::pair<float,Experiment*> > experiment_entropies){
    LightExperiment *target_experiment = NULL;
    //return;
/*
    target_experiment = (LightExperiment*)experiment_entropies[0].second;
    assert(target_experiment != NULL);

    target_experiment->print();

    ExperimentResult experiment_result = target_experiment->performVirtualExperiment(real_model, true);
    experiment_result.setLabel(ExperimentResultClassifier::instance().classifyExperimentResult(experiment_result));
    std::cout << "result: " << experiment_result.getLabel() << std::endl;

    updateModelPriors(experiment_result, target_experiment);
*/  

    unsigned num_features_detected = 0;
    bool success = false;
    Transform object_pose = Control::locateObject(obj, 2, num_features_detected, success);
    object_pose.shift.print();
    object_pose.mat.printOut();
    getchar();

/*
    Transform arm_to_object;
    bool success = false;

    while(true){
        arm_to_object = Control::pickUpObject(obj, success);
        if(success){
            break;
        }
        else{
            obj->invalidateTransform();
            std::cout << "Could not pick up object, please move it closer to the robot" << std::endl;
            std::cout << "Continue?" << std::endl;
            getchar();
        }

    }
    arm_to_object = Control::refineArmToObject(obj);

    float bottom_offset = 4.6;//real_model->getPhysicsObject()->getBoxBottomOffset();

    float target_experiment_entropy = 0.0f;
    float start_y = 0.0f;
    for(unsigned j = 0; j < experiment_entropies.size(); j++){
        if(Control::moveToExperiment(obj, arm_to_object, bottom_offset, ramp_normal, 
                                     (DropExperiment*)experiment_entropies[j].second, 0.3f, start_y)){
            target_experiment_entropy = experiment_entropies[j].first;
            target_experiment = (DropExperiment*)experiment_entropies[j].second;
            break;
        }
        else{
            std::cout << "Cannot perform experiment " << j << std::endl;
        }
    }

    std::cout << "Experiment:" << target_experiment->getDropHeight() << " " << target_experiment->getObjRotation() << std::endl;
    Sleep(2000);
    Arm::getArm()->releaseHand();
    Common::sleep(2000);
    Control::moveArmOutOfTheWay();

    unsigned num_features_detected = 0;
    bool locate_success = false;
    Transform result_pose = Control::locateObject(obj, 1, num_features_detected, locate_success);
    result_pose.shift.y -= start_y;
    result_pose.shift.x += 0.0f;

    std::cout << "Result shift:"; result_pose.shift.print();

    ExperimentResult experiment_result(result_pose);
    experiment_result.setLabel(ExperimentResultClassifier::instance().classifyExperimentResult(experiment_result));

    updateModelPriors(experiment_result, target_experiment);
    */
}

float ExperimentCoordinator::calculateExpectedKLDivergence(Experiment* experiment, std::vector<float> &prior){
    assert(experiment != NULL);

    unsigned num_labels = ExperimentResultClassifier::instance().getNumLabels();

    float result = 0.0f;
    for(unsigned i = 0; i < num_labels; i++){
        std::vector<float> object_pd;
        float weight = 0.0f;

        for(unsigned j = 0; j < all_phys_models.size(); j++){
            std::vector<float> pose_object_probabilities = 
                experiment->getConditionalResultProbabilities(num_labels, all_phys_models[j]);

            float p = (pose_object_probabilities[i]*cur_model_pd[j]);
            object_pd.push_back(p);

            weight += p;
        }

        Common::formProbabilityDistribution(object_pd);

        result += weight * Common::KLDivergence(object_pd, prior);
    }

    return result;
}

bool ExperimentCoordinator::isValidDistribution(const std::vector<float> &distribution){
    for(unsigned i = 0; i < distribution.size(); i++){
        if(distribution[i] != distribution[i]){
            return false;
        }

        if(distribution[i] < 0.0f || distribution[i] > 1.0f){
            return false;
        }
    }

    return true;
}

void ExperimentCoordinator::updateModelPriors(ExperimentResult experiment_result, Experiment *experiment){
    assert(experiment != NULL);
    assert(all_phys_models.size() == cur_model_pd.size());
    
    unsigned num_labels = ExperimentResultClassifier::instance().getNumLabels();
    std::vector<float> new_priors;
    for(unsigned i = 0; i < all_phys_models.size(); i++){
        std::vector<float> resultp = 
            experiment->getConditionalResultProbabilities(num_labels, all_phys_models[i]);

        float new_model_p = resultp[experiment_result.getLabel()] * cur_model_pd[i];
        
        const float epsilon = 0.00001f;
        if(new_model_p < epsilon){
            new_model_p = epsilon;
        }
        if(new_model_p > 1.0f-epsilon){
            new_model_p = 1.0f-epsilon;
        }

        new_priors.push_back(new_model_p);
    }

    Common::formProbabilityDistribution(new_priors);
    if(isValidDistribution(new_priors)){
        cur_model_pd = new_priors;
    }
    else{
        std::cout << "invalid distribution:" << std::endl;
        Common::printVector(new_priors);
    }
}
