
#include "ExperimentOptimiser.h"
#include "ExperimentCoordinator.h"
#include "../Util/Vector3D.h"
#include "../Util/Geometry.h"
#include "../OptimiserClass/Optimiser.h"
#include "../OptimiserClass/NelderMead.h"

#include <cassert>

static std::vector<float> orientationToParams(Vector3D orientation){
    orientation.normalise();

    std::vector<float> result;
    result.push_back(orientation.x/2.0f + 0.5f);
    result.push_back(orientation.y/2.0f + 0.5f);
    result.push_back(orientation.z/2.0f + 0.5f);
    return result;
}

static Vector3D paramsToOrientation(std::vector<float> params){
    Vector3D result;
    result.x = (params[0] - 0.5f)*2.0f;
    result.y = (params[1] - 0.5f)*2.0f;
    result.z = (params[2] - 0.5f)*2.0f;
    result.normalise();
    return result;
}

class ExperimentFitnessFunction : public OptimisableFunction {
  public:
    ExperimentFitnessFunction(){}

    float eval(std::vector<float> &params){
        return 0.0f; 
        /*
        Vector3D up_vec = paramsToOrientation(params);
        //up_vec.print();
        Matrix3 orientation_mat = Geometry::getMatrixFromTo(up_vec, Vector3D(0.0f, 0.0f, 1.0f));
        Quaternion orientation_quaternion(orientation_mat);
        Experiment *cur_experiment = new DropExperiment(0.5f, orientation_quaternion, up_vec);
        cur_experiment->setPhysicsWorld(ExperimentCoordinator::instance().getPhysWorld());

        std::vector<float> tmp;
        float result = ExperimentCoordinator::instance().calculateExpectedExprimentEntropy(cur_experiment, tmp);
        //std::cout << "r: " << result << std::endl;
        delete cur_experiment;
        return result;
        */
    }
};


DropExperiment* ExperimentOptimiser::optimise(DropExperiment* start_point){
    return NULL;
    /*
    assert(start_point != NULL);

    Vector3D start_orientation = start_point->getOrientation();
    //start_orientation.print();
    std::vector<float> tmp;
    //std::cout << "se: " << ExperimentCoordinator::instance().calculateExpectedExprimentEntropy(start_point, tmp) << std::endl;

    ExperimentFitnessFunction *fitness_function = new ExperimentFitnessFunction();
    NelderMead optimiser(orientationToParams(start_orientation));

    std::vector<float> optimised_result = optimiser.optimise(fitness_function, 3, 10);

    Vector3D up_vec = paramsToOrientation(optimised_result);
    Matrix3 orientation_mat = Geometry::getMatrixFromTo(up_vec, Vector3D(0.0f, 0.0f, 1.0f));
    Quaternion orientation_quaternion(orientation_mat);
    DropExperiment *best_experiment = new DropExperiment(0.5f, orientation_quaternion, up_vec);
    best_experiment->setPhysicsWorld(ExperimentCoordinator::instance().getPhysWorld());

    return best_experiment;
    */
}