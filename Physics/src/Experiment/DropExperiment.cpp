
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
#define ORIENTATION_SD (5.0f*M_PI/180.0f)
#define EXPERIMENT_RESULT_SD 7.0f

#define PDIST_FLOOR_HEIGHT 0.2f
#define NUM_EXPERIMENTS 70

static unsigned numNonZero(std::vector<float> &vec){
    unsigned result = 0;
    for(unsigned i = 0; i < vec.size(); i++){
        if(vec[i] > 0.0f){
            result++;
        }
    }
    return result;
}

static unsigned cur_id = 0;

DropExperiment::DropExperiment(float drop_height, float obj_rotation, Vector3D ground_plane_normal) :
    my_phys_world(NULL), 
    drop_height(drop_height), 
    obj_rotation(obj_rotation), 
    ground_plane_normal(ground_plane_normal) {

    id = cur_id++;
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

    std::cout << "Calculating conditional result probabilities" << std::endl;
    std::vector<float> result(num_possible_results, 0.0f);
    WheeledBoxPhysicsObject *phys_obj = model.getPhysicsObject();

    for(unsigned i = 0; i < NUM_EXPERIMENTS; i++){
        ExperimentResult er = performVirtualExperiment(phys_obj, NULL);
        er.setLabel(ExperimentResultClassifier::instance().classifyExperimentResult(er));

        addResult(er, result);
    }
    delete phys_obj;

    Common::formProbabilityDistribution(result);
    for(unsigned i = 0; i < result.size(); i++){
        result[i] += PDIST_FLOOR_HEIGHT/(float)result.size();
    }
    Common::formProbabilityDistribution(result);

    cached_object_conditional_probabilities[model.getId()] = result;
    return result;
}

ExperimentResult DropExperiment::performExperiment(void){
    ExperimentResult result;
    return result;
}

//#define VISUALISE_EXPERIMENT
ExperimentResult DropExperiment::performVirtualExperiment(WheeledBoxPhysicsObject *wbox, 
                                                          WheeledBoxRenderObject *render_obj){
    assert(phys_obj != NULL);

    float cur_drop_height = getNoisyHeight(drop_height);
    float cur_orientation = getNoisyOrientation(obj_rotation);

    float theta = acosf(ground_plane_normal.dotProduct(Vector3D(0.0f, 0.0f, 1.0f)));
    Vector3D start_pos = Vector3D(-cur_drop_height/tanf(theta), 30.0f, cur_drop_height + GROUND_HEIGHT);
    start_pos = start_pos + wbox->getBoxBottomOffset()*ground_plane_normal;

    Matrix3 rot_mat = Geometry::axisRotationMatrix(ground_plane_normal, cur_orientation);

    Transform new_pose;
    new_pose.mat = rot_mat * Geometry::getMatrixFromTo(Vector3D(0.0f, 0.0f, 1.0f), ground_plane_normal);
    new_pose.quaternion = Quaternion(new_pose.mat);
    new_pose.shift = start_pos;
    
    my_phys_world->addObject(wbox);

    wbox->moveTo(new_pose);
    wbox->activate();
    //wbox->perturbParams();

    if(render_obj != NULL){
        render_obj->setBoxTransform(wbox->getBoxTransform());
        render_obj->setStubTransforms(wbox->getStubTransforms());
        render_obj->setWheelTransforms(wbox->getWheelTransforms());

        SceneRenderer::instance().addObject(render_obj);
    }
    
    // Run the simulation loop a bunch of times.
    unsigned num_iterations = 0;

    while(num_iterations < 600){
        wbox->update();

        if(render_obj != NULL){
            render_obj->setBoxTransform(wbox->getBoxTransform());
            render_obj->setStubTransforms(wbox->getStubTransforms());
            render_obj->setWheelTransforms(wbox->getWheelTransforms());
            Sleep(20);
        }
        /*
        if(num_iterations%30 == 0){
            applyNoisyImpulse(wbox, 2.0f);
        }
        */

        my_phys_world->step();
        num_iterations++;
    }

    Transform result_pose = wbox->getBoxTransform();
    result_pose.shift.y -= start_pos.y;

    my_phys_world->removeObject(wbox->getId());

    if(render_obj != NULL){
        SceneRenderer::instance().removeObject(render_obj->getId());
    }

    return ExperimentResult(result_pose);
}

bool DropExperiment::canPerformExperiment(Transform arm_to_obj){
    return true;
}

void DropExperiment::setPhysicsWorld(PhysicsWorld *phys_world){
    assert(phys_world != NULL);
    my_phys_world = phys_world;
}

void DropExperiment::print(void){
    std::cout << "Experiment id: " << id << std::endl;
}

void DropExperiment::save(std::ostream &out_stream){
    out_stream << drop_height << " " 
               << obj_rotation << std::endl;

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
    in_stream >> obj_rotation;

    unsigned num;

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

void DropExperiment::addResult(ExperimentResult result, std::vector<float> &distribution){
    for(unsigned i = 0; i < distribution.size(); i++){
        ExperimentResult prototype = ExperimentResultClassifier::instance().getPrototypeResult(i);
        float d = prototype.distance(result);
        float weight = Common::normalDistribution(0.0f, EXPERIMENT_RESULT_SD, d);
        distribution[i] += weight;
    }
}

float DropExperiment::getNoisyHeight(float base_height){
    return base_height + Common::gaussianNoise(0.0f, DROP_HEIGHT_SD);
}

float DropExperiment::getNoisyOrientation(float base_orientation){
    float result = base_orientation + Common::gaussianNoise(0.0f, ORIENTATION_SD);
    while(result < -M_PI){
        result += 2.0f*M_PI;
    }
    while(result > M_PI){
        result -= 2.0f*M_PI;
    }
    return result;
}

void DropExperiment::applyNoisyImpulse(WheeledBoxPhysicsObject* phys_obj, float amount){
    Vector3D impulse_vec((rand()/(float)RAND_MAX-0.5f)*2.0f, (rand()/(float)RAND_MAX-0.5f)*2.0f, 0.0f);
    impulse_vec.normalise();
    impulse_vec.scale(amount);

    phys_obj->applyImpulse(impulse_vec, Vector3D(0.0f, 0.0f, 1.0f));
}