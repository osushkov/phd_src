
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
    DropExperiment(float drop_height, float obj_rotation, Vector3D ground_plane_normal);
    ~DropExperiment();

    ExperimentType getType(void) const;

    std::vector<float> getConditionalResultProbabilities(unsigned num_possible_results, ObjectPhysModel model);

    ExperimentResult performExperiment(void);
    ExperimentResult performVirtualExperiment(WheeledBoxPhysicsObject *wbox, 
                                              WheeledBoxRenderObject *render_obj=NULL);

    bool canPerformExperiment(Transform arm_to_obj);

    void setPhysicsWorld(PhysicsWorld *phys_world);
    void print(void);

    void save(std::ostream &out_stream);
    void load(std::istream &in_stream);

    float getDropHeight(void){ return drop_height; }
    float getObjRotation(void){ return obj_rotation; }

  private:
    unsigned id;
    PhysicsWorld *my_phys_world;

    float drop_height;
    float obj_rotation;
    
    Vector3D ground_plane_normal;

    void addResult(ExperimentResult result, std::vector<float> &distribution);

    float getNoisyHeight(float base_height);
    float getNoisyOrientation(float base_orientation);
    void applyNoisyImpulse(WheeledBoxPhysicsObject* phys_obj, float amount);
};


#endif
