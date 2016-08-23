
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

    model_pd_file << std::endl << std::endl;
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
        std::cout << "calculating experiment entropies" << std::endl;
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
    
    real_model = ObjectLegoModel(0, 1, -1, 0, 
        std::pair<Vector3D,Vector3D>(Vector3D(-5.0f, 6.0f, -1.5f), Vector3D(-0.707f, 0.0f, -1.0)),
        std::pair<Vector3D,Vector3D>(Vector3D(-5.0f, -6.0f, -1.5f), Vector3D(-0.707f, 0.0f, -1.0)));
}

void ExperimentCoordinator::performWorldExperiment(Object *obj, std::vector<std::pair<float,Experiment*> > experiment_entropies){
    LightExperiment *target_experiment = NULL;
    //return;

    target_experiment = (LightExperiment*)experiment_entropies[0].second;
    assert(target_experiment != NULL);

    target_experiment->print();

    ExperimentResult experiment_result = target_experiment->performVirtualExperiment(real_model, true);
    
    experiment_result.setLabel(ExperimentResultClassifier::instance().classifyExperimentResult(experiment_result));
    std::cout << "result: " << experiment_result.getOrientation() << " : "; experiment_result.getPosition().print();

    updateModelPriors(experiment_result, target_experiment);


    /*
    std::cout << "replace object" << std::endl;
    getchar();
    
    unsigned num_features_detected = 0;
    bool success = false;

    obj->invalidateTransform();
    Transform object_pose = Control::locateObject(obj, 3, num_features_detected, success);

    for(unsigned i = 0; i < experiment_entropies.size(); i++){
        target_experiment = (LightExperiment*)experiment_entropies[0].second;
        if(Control::moveToExperiment(obj, object_pose, 12.0f, target_experiment, 0.0f)){
            break;
        }
        target_experiment = NULL;
    }

    if(target_experiment == NULL){
        std::cerr << "Could not perform any experiments" << std::endl;
        return;
    }

    std::cout << "target experiment radius: " << target_experiment->getLightRadius() << std::endl;

    Common::sleep(2000);
    Control::moveArmOutOfTheWay();

    Matrix3 inv_mat;
    inv_mat.isInverse(object_pose.mat);

    obj->invalidateTransform();
    Transform result_pose = Control::locateObject(obj, 3, num_features_detected, success);
    result_pose.shift = inv_mat * (result_pose.shift - object_pose.shift);
    result_pose.mat = inv_mat * result_pose.mat;

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
