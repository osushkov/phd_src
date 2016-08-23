
#include "DropExperiment.h"
#include "ExperimentResultClassifier.h"
#include "../Visualisation/MeshRenderObject.h"
#include "../Visualisation/SceneRenderer.h"
#include "../Util/Geometry.h"
#include "../Util/Quaternion.h"
#include "../Util/Timer.h"

#include "../Physics/PhysicsWorld.h"
#include "../Physics/PhysicsObject.h"
#include "../Physics/ConvexHullPhysicsObject.h"

#include <cassert>
#include <cstdio>

#define _USE_MATH_DEFINES
#include <math.h>

#define DROP_HEIGHT_SD 0.5f
#define DROP_ORI_SD 10.0f
#define PDIST_FLOOR_HEIGHT 0.05f
#define NUM_EXPERIMENTS 50

static unsigned numNonZero(std::vector<float> &vec){
    unsigned result = 0;
    for(unsigned i = 0; i < vec.size(); i++){
        if(vec[i] > 0.0f){
            result++;
        }
    }
    return result;
}

DropExperiment::DropExperiment(float drop_height, Quaternion orientation_quaternion, Vector3D up_vec) :
    my_phys_world(NULL), drop_height(drop_height), orientation_quaternion(orientation_quaternion), up_vec(up_vec) {

}

DropExperiment::~DropExperiment(){

}

ExperimentType DropExperiment::getType(void) const {
    return DROP_EXPERIMENT;
}

std::vector<float> DropExperiment::getConditionalResultProbabilities(unsigned num_possible_results, ObjectPhysModel model){
	std::map<unsigned, std::vector<float> >::iterator it = cached_object_conditional_probabilities.find(model.getId());
	if(it != cached_object_conditional_probabilities.end()){
		return it->second;
	}

    std::vector<float> result(num_possible_results, 0.0f);
    PhysicsObject *phys_obj = model.getPhysicsObject();

    for(unsigned i = 0; i < NUM_EXPERIMENTS; i++){
        ExperimentResult er = performVirtualExperiment(phys_obj, NULL);
        er.setLabel(ExperimentResultClassifier::instance().classifyExperimentResult(er));
        result[er.getLabel()] += 1.0f;
    }
    delete phys_obj;

    Common::formProbabilityDistribution(result);
    for(unsigned i = 0; i < result.size(); i++){
        result[i] += PDIST_FLOOR_HEIGHT;
    }
    Common::formProbabilityDistribution(result);

    cached_object_conditional_probabilities[model.getId()] = result;
    return result;
}

std::vector<float> DropExperiment::getResultProbabilities(unsigned num_possible_results, std::vector<ObjectPhysModel> all_models){
	if(cached_result_probabilities.size() > 0){
		return cached_result_probabilities;
	}

    std::vector<float> result(num_possible_results, 0.0f);
    for(unsigned i = 0; i < all_models.size(); i++){
        PhysicsObject* phys_obj = all_models[i].getPhysicsObject();
        RenderObject* render_obj = NULL; //all_models[i].getRenderObject();

        for(unsigned j = 0; j < NUM_EXPERIMENTS; j++){
            ExperimentResult er = performVirtualExperiment(phys_obj, render_obj);
            er.setLabel(ExperimentResultClassifier::instance().classifyExperimentResult(er));
            result[er.getLabel()] += 1.0f;
        }

        if(render_obj != NULL){
            delete render_obj;
        }
        delete phys_obj;
    }

    Common::formProbabilityDistribution(result);
    for(unsigned i = 0; i < result.size(); i++){
        result[i] += PDIST_FLOOR_HEIGHT;
    }
    Common::formProbabilityDistribution(result);

    cached_result_probabilities = result;
    return result;
}

ExperimentResult DropExperiment::performExperiment(void){
    ExperimentResult result(Vector3D(0.0f, 0.0f, 0.0f), 0);
    return result;
}

//#define VISUALISE_EXPERIMENT
ExperimentResult DropExperiment::performVirtualExperiment(PhysicsObject* phys_obj, RenderObject *render_obj){
    assert(phys_obj != NULL);
    my_phys_world->clear(); 

    if(render_obj != NULL){
        SceneRenderer::instance().initialise(false);
        SceneRenderer::instance().clearObjects();
        SceneRenderer::instance().addObject(render_obj);
    }

    float bottom_height = getNoisyHeight(drop_height);
    if(bottom_height < 0.1f){ bottom_height = 0.1f; }

    Transform new_pose;
    new_pose.quaternion = getNoisyOrientation(orientation_quaternion);
    float lowest_point = phys_obj->getLowestSurfacePoint(new_pose.quaternion);
    new_pose.shift = Vector3D(0.0f, 0.0f, bottom_height - lowest_point);

    my_phys_world->addObject(phys_obj);
    phys_obj->setPose(new_pose);
    phys_obj->activate();
    
    // Run the simulation loop a bunch of times.
    unsigned num_iterations = 0;

    for(unsigned i = 0; i < 400; i++){
        assert(phys_obj->isActive());

        if(render_obj != NULL){
            Transform cur_transform = phys_obj->getTransform();
            render_obj->setTransform(cur_transform);
            SceneRenderer::instance().signal();
        }

        if(i%30 == 0 && i > 0){
            applyNoisyImpulse(phys_obj, 2.0f);
        }

        my_phys_world->step();
        num_iterations++;
    }
/*
    phys_obj->activate();

    Vector3D impulse_vec((rand()/(float)RAND_MAX-0.5f)*2.0f, (rand()/(float)RAND_MAX-0.5f)*2.0f, 0.0f);
    impulse_vec.normalise();
    impulse_vec.scale(10.0f);
    
    phys_obj->applyImpulse(impulse_vec, Vector3D(0.0f, 0.0f, 1.0f));
    while(phys_obj->isActive()){
        if(render_obj != NULL){
            Transform cur_transform = phys_obj->getTransform();
            render_obj->setTransform(cur_transform);
            SceneRenderer::instance().signal();
        }

        my_phys_world->step();
        num_iterations++;
        //std::cout << "impoulse" << std::endl;
    }
    */

    Transform result_pose = phys_obj->getTransform();
    my_phys_world->clear();

    return ExperimentResult(result_pose);
}

bool DropExperiment::canPerformExperiment(Transform arm_to_obj){
    Vector3D arm_forward_vec(0.0f, 0.0f, 1.0f);
    Vector3D obj_space = arm_to_obj.mat*arm_forward_vec;

    if(obj_space.dotProduct(up_vec) > 0.2){
        return false;
    }
    else{
        return true;
    }
}

void DropExperiment::setPhysicsWorld(PhysicsWorld *phys_world){
    assert(phys_world != NULL);
    my_phys_world = phys_world;
}

void DropExperiment::print(void){
    std::cout << "DROP: " << drop_height << " , ";
    up_vec.print();
}

void DropExperiment::save(std::ostream &out_stream){
    out_stream << drop_height << " " 
               << orientation_quaternion.w << " "
               << orientation_quaternion.x << " "
               << orientation_quaternion.y << " "
               << orientation_quaternion.z << " "
               << up_vec.x << " "
               << up_vec.y << " "
               << up_vec.z << std::endl;

    out_stream << cached_result_probabilities.size() << std::endl;
    for(unsigned i = 0; i < cached_result_probabilities.size(); i++){
        out_stream << cached_result_probabilities[i] << std::endl;
    }

    out_stream << cached_object_conditional_probabilities.size() << std::endl;

    std::map<unsigned, std::vector<float> >::iterator it;
    for(it = cached_object_conditional_probabilities.begin();
        it != cached_object_conditional_probabilities.end();
        ++it){

        out_stream << it->first << " " << it->second.size() << std::endl;
        for(unsigned i = 0; i < it->second.size(); i++){
            out_stream << it->second[i] << std::endl;
        }
    }
}

void DropExperiment::load(std::istream &in_stream){
    in_stream >> drop_height;
    in_stream >> orientation_quaternion.w;
    in_stream >> orientation_quaternion.x;
    in_stream >> orientation_quaternion.y;
    in_stream >> orientation_quaternion.z;
    in_stream >> up_vec.x;
    in_stream >> up_vec.y;
    in_stream >> up_vec.z;

    unsigned num;
    in_stream >> num;
    
    cached_result_probabilities.clear();
    for(unsigned i = 0; i < num; i++){
        float p;
        in_stream >> p;
        cached_result_probabilities.push_back(p);
    }

    in_stream >> num;
    cached_object_conditional_probabilities.clear();
    for(unsigned i = 0; i < num; i++){
        unsigned key;
        in_stream >> key;

        std::vector<float> vals;
        unsigned num_vals;
        in_stream >> num_vals;
        for(unsigned j = 0; j < num_vals; j++){
            float p;
            in_stream >> p;
            vals.push_back(p);
        }

        cached_object_conditional_probabilities.insert(std::pair<unsigned,std::vector<float> >(key, vals));
    }
}

float DropExperiment::getNoisyHeight(float base_height){
    return base_height + Common::gaussianNoise(0.0f, DROP_HEIGHT_SD);
}

Quaternion DropExperiment::getNoisyOrientation(Quaternion base_orientation){
    //return base_orientation;
    Matrix3 orientation_matrix = base_orientation.toMatrix();

    Vector3D rotation_axis((rand()/(float)RAND_MAX-0.5f)*2.0f, (rand()/(float)RAND_MAX-0.5f)*2.0f, (rand()/(float)RAND_MAX-0.5f)*2.0f);
    while(rotation_axis.length() > 1.0f){
        rotation_axis = Vector3D((rand()/(float)RAND_MAX-0.5f)*2.0f, (rand()/(float)RAND_MAX-0.5f)*2.0f, (rand()/(float)RAND_MAX-0.5f)*2.0f);
    }
    rotation_axis.normalise();

    float theta = Common::gaussianNoise(0.0f, DROP_ORI_SD*(float)M_PI/180.0f);
    Matrix3 perturb_mat = Geometry::axisRotationMatrix(rotation_axis, theta);
    Matrix3 result = perturb_mat * orientation_matrix;

    return Quaternion(result);
}

void DropExperiment::applyNoisyImpulse(PhysicsObject* phys_obj, float amount){
    Vector3D impulse_vec((rand()/(float)RAND_MAX-0.5f)*2.0f, (rand()/(float)RAND_MAX-0.5f)*2.0f, 0.0f);
    impulse_vec.normalise();
    impulse_vec.scale(amount);

    phys_obj->applyImpulse(impulse_vec, Vector3D(0.0f, 0.0f, 1.0f));
}