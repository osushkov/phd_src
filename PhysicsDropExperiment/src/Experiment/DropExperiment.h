
#ifndef _DropExperiment_H_
#define _DropExperiment_H_

#include "Experiment.h"
#include "ExperimentResult.h"
#include "../Visualisation/MeshRenderObject.h"
#include "../Visualisation/SceneRenderer.h"
#include "../Util/Quaternion.h"
#include "../Util/Semaphore.h"

#include <vector>

class DropExperiment : public Experiment {
  public:
    DropExperiment(float drop_height, Quaternion orientation_quaternion, Vector3D up_vec);
    ~DropExperiment();

    ExperimentType getType(void) const;

    std::vector<float> getConditionalResultProbabilities(unsigned num_possible_results, ObjectPhysModel model);
    std::vector<float> getResultProbabilities(unsigned num_possible_results, std::vector<ObjectPhysModel> all_models);

    ExperimentResult performExperiment(void);
    ExperimentResult performVirtualExperiment(PhysicsObject* phys_obj, RenderObject *render_obj=NULL);

    bool canPerformExperiment(Transform arm_to_obj);

    void setPhysicsWorld(PhysicsWorld *phys_world);
    void print(void);

    void save(std::ostream &out_stream);
    void load(std::istream &in_stream);

    Vector3D getUpVector(void){ return up_vec; }
    float getDropHeight(void){ return drop_height; }

  private:
    PhysicsWorld *my_phys_world;

    float drop_height;
    Quaternion orientation_quaternion;
    Vector3D up_vec;


    float getNoisyHeight(float base_height);
    Quaternion getNoisyOrientation(Quaternion base_orientation);
    void applyNoisyImpulse(PhysicsObject* phys_obj, float amount);
};


#endif
