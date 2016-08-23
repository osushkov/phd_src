
#include "LightExperiment.h"
#include "ExperimentResultClassifier.h"
#include "../Visualisation/LightBeamRenderObject.h"
#include "../Visualisation/BoxRenderObject.h"
#include "../Visualisation/SceneRenderer.h"
#include "../Util/Geometry.h"
#include "../Util/Quaternion.h"
#include "../Util/Timer.h"

#include <cassert>
#include <cstdio>

#define _USE_MATH_DEFINES
#include <math.h>

#define LIGHT_POS_SD 0.5f
#define LIGHT_RADIUS_SD 0.5f

#define ROBOT_TURN_SD 0.2f
#define ROBOT_RUN_SD 1.0f

#define EXPERIMENT_RESULT_SD 3.0f

#define PDIST_FLOOR_HEIGHT 0.1f
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

static unsigned cur_id = 0;

LightExperiment::LightExperiment(Vector3D light_pos, float light_radius) :
    light_pos(light_pos), light_radius(light_radius) {

    id = cur_id++;
}

LightExperiment::~LightExperiment(){

}

ExperimentType LightExperiment::getType(void) const {
    return LIGHT_EXPERIMENT;
}

std::vector<float> LightExperiment::getConditionalResultProbabilities(unsigned num_possible_results, ObjectLegoModel model){
	std::map<unsigned, std::vector<float> >::iterator it = cached_object_conditional_probabilities.find(model.getId());
	if(it != cached_object_conditional_probabilities.end()){
		return it->second;
	}

    std::cout << "Calculating conditional result probabilities" << std::endl;
    std::vector<float> result(num_possible_results, 0.0f);

    for(unsigned i = 0; i < NUM_EXPERIMENTS; i++){
        ExperimentResult er = performVirtualExperiment(model, false);
        er.setLabel(ExperimentResultClassifier::instance().classifyExperimentResult(er));

        addResult(er, result);
    }

    Common::formProbabilityDistribution(result);
    for(unsigned i = 0; i < result.size(); i++){
        result[i] += PDIST_FLOOR_HEIGHT/(float)result.size();
    }
    Common::formProbabilityDistribution(result);

    cached_object_conditional_probabilities[model.getId()] = result;
    return result;
}

ExperimentResult LightExperiment::performExperiment(void){
    ExperimentResult result;
    return result;
}

//#define VISUALISE_EXPERIMENT
ExperimentResult LightExperiment::performVirtualExperiment(ObjectLegoModel model, bool render){
    Vector3D offset_light_pos = getNoisyLightPos(light_pos);
    float offset_light_radius = getNoisyLightRadius(light_radius);

    float left_sensor_response = model.lightAmountLeftSensor(offset_light_pos - Vector3D(0.0f, 0.0f, 5.0f), offset_light_radius);
    float right_sensor_response = model.lightAmountRightSensor(offset_light_pos - Vector3D(0.0f, 0.0f, 5.0f), offset_light_radius);

    int turn_direction = 0;
    int run_direction = 0;

    const float MIN_SENSOR_RESPONSE = 0.1f;
    if(left_sensor_response > right_sensor_response && left_sensor_response > MIN_SENSOR_RESPONSE){
        turn_direction = model.getLeftSensorTurnDirection();
        run_direction = model.getLeftSensorRunDirection();
    }
    else if(right_sensor_response > MIN_SENSOR_RESPONSE){
        turn_direction = model.getRightSensorTurnDirection();
        run_direction = model.getRightSensorRunDirection();
    }

    LightBeamRenderObject *light_beam_ro = NULL;
    if(render){
        light_beam_ro = new LightBeamRenderObject(light_pos + Vector3D(0.0f, 0.0f, GROUND_HEIGHT), light_radius);
        SceneRenderer::instance().addObject(light_beam_ro);
    }

    std::vector<Vector3D> sensor_pos;
    sensor_pos.push_back(model.getLeftSensorPos());
    sensor_pos.push_back(model.getRightSensorPos());

    std::vector<Vector3D> sensor_dir;
    sensor_dir.push_back(model.getLeftSensorViewDir());
    sensor_dir.push_back(model.getRightSensorViewDir());

    BoxRenderObject *lego_ro = NULL;
    
    if(render){
        lego_ro = new BoxRenderObject(13.5f, 10.0f, 10.0f, sensor_pos, sensor_dir);
        SceneRenderer::instance().addObject(lego_ro);
    }

    Vector3D start_forward_vec(1.0f, 0.0f, 0.0f);
    Transform start_transform;
    start_transform.mat.identity();
    start_transform.shift = Vector3D(0.0f, 0.0f, GROUND_HEIGHT + 5.0f);

    float turn_amount = 45.0f * turn_direction * M_PI/180.0f;
    float run_amount = 15.0f * run_direction;

    if(fabs(turn_amount) > 0.01f){
        turn_amount +=  Common::gaussianNoise(0.0f, ROBOT_TURN_SD);
    }

    if(fabs(run_amount) > 0.01f){
        run_amount +=  Common::gaussianNoise(0.0f, ROBOT_RUN_SD);
    }

    const unsigned turn_frames = 30.0f;
    const unsigned run_frames = 30.0f;

    Transform cur_transform = start_transform;

    if(render){
        for(unsigned i = 0; i <= turn_frames; i++){
            float cur_turn = (float)i/(float)turn_frames * turn_amount;
            
            Matrix3 rot_mat = Geometry::axisRotationMatrix(Vector3D(0.0f, 0.0f, 1.0f), cur_turn);
            cur_transform.mat = rot_mat * start_transform.mat;

            lego_ro->setTransform(cur_transform);
            Sleep(50);
        }
    }
    else{
        cur_transform.mat = Geometry::axisRotationMatrix(Vector3D(0.0f, 0.0f, 1.0f), turn_amount) * start_transform.mat;
    }

    Matrix3 rot_mat = Geometry::axisRotationMatrix(Vector3D(0.0f, 0.0f, 1.0f), turn_amount);
    Vector3D forward_vec = rot_mat * start_forward_vec;

    if(render){
        for(unsigned i = 0; i  < run_frames; i++){
            cur_transform.shift = cur_transform.shift + run_amount/(float)run_frames * forward_vec;

            lego_ro->setTransform(cur_transform);
            Sleep(50);
        }
    }
    else{
        cur_transform.shift = cur_transform.shift + run_amount * forward_vec;
    }

    if(render){
        SceneRenderer::instance().removeObject(light_beam_ro->getId());
        SceneRenderer::instance().removeObject(lego_ro->getId());
        
        delete light_beam_ro;
        delete lego_ro;
    }

    cur_transform.shift.z = 0.0f;
    return ExperimentResult(cur_transform);
}

bool LightExperiment::canPerformExperiment(Transform arm_to_obj){
    return true;
}

void LightExperiment::print(void){
    std::cout << "Experiment id: " << id << std::endl;
}

void LightExperiment::save(std::ostream &out_stream){
    out_stream << light_pos.x << " " << light_pos.y << " " << light_pos.z << std::endl;
    out_stream << light_radius << std::endl;

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

void LightExperiment::load(std::istream &in_stream){
    in_stream >> light_pos.x;
    in_stream >> light_pos.y;
    in_stream >> light_pos.z;

    in_stream >> light_radius;

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

void LightExperiment::addResult(ExperimentResult result, std::vector<float> &distribution){
    for(unsigned i = 0; i < distribution.size(); i++){
        ExperimentResult prototype = ExperimentResultClassifier::instance().getPrototypeResult(i);
        float d = prototype.distance(result);
        float weight = Common::normalDistribution(0.0f, EXPERIMENT_RESULT_SD, d);
        distribution[i] += weight;
    }
}


Vector3D LightExperiment::getNoisyLightPos(Vector3D base_light_pos){
    Vector3D offset;

    while(true){
        offset.x = (2.0f*(float)rand()/(float)RAND_MAX) - 1.0f;
        offset.y = (2.0f*(float)rand()/(float)RAND_MAX) - 1.0f;
        offset.z = 0.0f;
    
        if(offset.length() < 1.0f){
            break;
        }
    }

    offset.normalise();
    offset.scale(Common::gaussianNoise(0.0f, LIGHT_POS_SD));

    return base_light_pos + offset;
}

float LightExperiment::getNoisyLightRadius(float base_light_radius){
    return base_light_radius + Common::gaussianNoise(0.0f, LIGHT_RADIUS_SD);
}
