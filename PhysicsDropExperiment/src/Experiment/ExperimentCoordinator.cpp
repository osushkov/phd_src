
#include "ExperimentCoordinator.h"
#include "ExperimentOptimiser.h"
#include "ExperimentResultClassifier.h"
#include "../Control.h"
#include "../Arm/Arm.h"
#include "../Util/Common.h"
#include "../Util/Timer.h"
#include "../Util/ParallelServer.h"
#include <cstdio>

#define _USE_MATH_DEFINES
#include <math.h>

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

	delete phys_world;

}

void ExperimentCoordinator::initialise(std::vector<ObjectPhysModel> phys_models, std::vector<Experiment*> experiments){
    if(is_initialised){
        return;
    }

    phys_world = new PhysicsWorld(0);
    phys_world->initialise();

    all_experiments = experiments;
    for(unsigned i = 0; i < all_experiments.size(); i++){
    	all_experiments[i]->setPhysicsWorld(phys_world);
    }

    all_phys_models = phys_models;
    /*
    cur_model_pd.push_back(0.995f);
    cur_model_pd.push_back(0.002f);
    cur_model_pd.push_back(0.001f);
    */
    for(unsigned i = 0; i < all_phys_models.size(); i++){
        cur_model_pd.push_back(1.0f/all_phys_models.size());
    }
    
    Common::formProbabilityDistribution(cur_model_pd);

    // This is where for testing I create the actual model it is trying to find.
    SuperQuadric shape(0.1f, 1.0f, 2.5f, 2.5f, 7.0f);
    real_model = ObjectPhysModel(shape, 10.0f, Vector3D(0.0f, 0.0f, 3.5f), 0.1f);
    //SuperQuadric shape(0.1f, 0.1f, 2.75f, 4.2f, 5.95f);
    //real_model = ObjectPhysModel(shape, 10.0f, Vector3D(0.0f, 0.0f, 3.0f), 0.1f);

    if(!ExperimentResultClassifier::instance().isBuilt()){
        std::cout << "Generating possible experiment results" << std::endl;
        std::vector<ExperimentResult> results_seen;
        for(unsigned i = 0; i < all_experiments.size(); i++){
            for(unsigned j = 0; j < phys_models.size(); j++){

                PhysicsObject *phys_object = phys_models[j].getPhysicsObject();
                RenderObject *render_object = NULL; //phys_models[j].getRenderObject();
                for(unsigned k = 0; k < 50; k++){
                    ExperimentResult eresult = all_experiments[i]->performVirtualExperiment(phys_object, render_object);
                    results_seen.push_back(eresult);
                }
                delete phys_object;
            }
        }

        ExperimentResultClassifier::instance().buildClassifier(results_seen);
    }

    is_initialised = true;
}

struct EntropyComp {
    EntropyComp(float cur_entropy) : cur_entropy(cur_entropy) {}

    bool operator()(const std::pair<float,Experiment*> &a, const std::pair<float,Experiment*> &b){
        //return fabs(cur_entropy-a.first) > fabs(cur_entropy-b.first);
        return a.first > b.first;
    }

    const float cur_entropy;
};

void ExperimentCoordinator::run(Object *obj){
    float cur_entropy = Common::distributionEntropy(cur_model_pd);

    for(unsigned experiment_iter = 0; experiment_iter < 20; experiment_iter++){
        std::cout << "Before: ";
        Common::printVector(cur_model_pd);
        std::vector<std::pair<float,Experiment*> > experiment_entropies;
        for(unsigned i = 0; i < all_experiments.size(); i++){
        	//std::vector<float> object_pd;
        	//float exp_entropy = calculateExpectedExprimentEntropy(all_experiments[i], object_pd);
            //experiment_entropies.push_back(std::pair<float,Experiment*>(exp_entropy, all_experiments[i]));
            float exp_info = calculateExpectedKLDivergence(all_experiments[i], cur_model_pd);
            experiment_entropies.push_back(std::pair<float,Experiment*>(exp_info, all_experiments[i]));
        }

        EntropyComp comp(cur_entropy);
        std::sort(experiment_entropies.begin(), experiment_entropies.end(), comp);
        assert(experiment_entropies.front().first <= experiment_entropies.back().first);

        performWorldExperiment(obj, experiment_entropies);
        cur_entropy = Common::distributionEntropy(cur_model_pd);

        std::cout << "After: ";
        Common::printVector(cur_model_pd);
    }
}

void ExperimentCoordinator::performWorldExperiment(Object *obj, std::vector<std::pair<float,Experiment*> > experiment_entropies){
    
    DropExperiment *target_experiment = NULL;

    target_experiment = (DropExperiment*)experiment_entropies[0].second;
    assert(target_experiment != NULL);

    PhysicsObject *real_phys_obj = real_model.getPhysicsObject();
    RenderObject *real_render_obj = real_model.getRenderObject();

    ExperimentResult experiment_result = target_experiment->performVirtualExperiment(real_phys_obj, real_render_obj);
    experiment_result.setLabel(ExperimentResultClassifier::instance().classifyExperimentResult(experiment_result));
    std::cout << "result: " << experiment_result.getLabel() << std::endl;

    if(real_phys_obj != NULL){ delete real_phys_obj; }
    //if(real_render_obj != NULL){ delete real_render_obj; }

    updateModelPriors(experiment_result, target_experiment);
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

    std::cout << "Best entropy: " << experiment_entropies.front().first << std::endl;

    float target_experiment_entropy = 0.0f;
    for(unsigned j = 0; j < experiment_entropies.size(); j++){
        if(experiment_entropies[j].second->canPerformExperiment(arm_to_object) && 
           Control::moveToExperiment(obj, arm_to_object, (DropExperiment*)experiment_entropies[j].second, 0.3f)){
            target_experiment_entropy = experiment_entropies[j].first;
            target_experiment = (DropExperiment*)experiment_entropies[j].second;
            break;
        }
        else{
            std::cout << "Cannot perform experiment " << j << std::endl;
        }
    }

    Sleep(2000);
    Arm::getArm()->releaseHand();
    Common::sleep(2000);
    Control::moveArmOutOfTheWay();

    unsigned num_features_detected = 0;
    bool locate_success = false;
    Transform result_pose = Control::locateObject(obj, 1, num_features_detected, locate_success);

    ExperimentResult experiment_result(result_pose);
    experiment_result.setLabel(ExperimentResultClassifier::instance().classifyExperimentResult(experiment_result));

    updateModelPriors(experiment_result, target_experiment);
    */
}

float ExperimentCoordinator::calculateExpectedExprimentEntropy(Experiment* experiment, std::vector<float> &object_pd){
    /*
    assert(experiment != NULL);
    object_pd.clear(); // resulting per-object probabiliy distribution for this experiment.
    unsigned num_labels = ExperimentResultClassifier::instance().getNumLabels();
    std::vector<float> result_probabilities = experiment->getResultProbabilities(num_labels, all_phys_models);

    for(unsigned i = 0; i < all_phys_models.size(); i++){
        float cur_object_probability = 0.0f;
        
        // Calculate here P(CurObject|CurPose) = P(CurPose|CurObject)*P(CurObject)/P(CurPose)
        std::vector<float> pose_object_probabilities = 
            experiment->getConditionalResultProbabilities(num_labels, all_phys_models[i]);

        for(unsigned j = 0; j < pose_object_probabilities.size(); j++){
        	if(result_probabilities[j] > 0.01f){
        		cur_object_probability += (pose_object_probabilities[j]*cur_model_pd[i])*result_probabilities[j];
        	}
        }

        object_pd.push_back(cur_object_probability);
    }

    Common::formProbabilityDistribution(object_pd);
    return Common::distributionEntropy(object_pd);
    */

    assert(experiment != NULL);
    object_pd.clear(); // resulting per-object probabiliy distribution for this experiment.
    unsigned num_labels = ExperimentResultClassifier::instance().getNumLabels();
    std::vector<float> result_probabilities = experiment->getResultProbabilities(num_labels, all_phys_models);

    float result = 0.0f;
    for(unsigned i = 0; i < result_probabilities.size(); i++){
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

        result += weight * Common::distributionEntropy(object_pd);
        //result += result_probabilities[i]*Common::distributionEntropy(object_pd);
    }

    return result;
}

float ExperimentCoordinator::calculateExpectedKLDivergence(Experiment* experiment, std::vector<float> &prior){
    assert(experiment != NULL);

    unsigned num_labels = ExperimentResultClassifier::instance().getNumLabels();
    std::vector<float> result_probabilities = experiment->getResultProbabilities(num_labels, all_phys_models);

    float result = 0.0f;
    for(unsigned i = 0; i < result_probabilities.size(); i++){
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
        //result += result_probabilities[i]*Common::distributionEntropy(object_pd);
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
