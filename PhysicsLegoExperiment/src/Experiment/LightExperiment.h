#ifndef _LightExperiment_H_
#define _LightExperiment_H_

#include "Experiment.h"
#include "ExperimentResult.h"
#include "../Visualisation/MeshRenderObject.h"
#include "../Visualisation/SceneRenderer.h"
#include "../Util/Quaternion.h"
#include "../Util/Semaphore.h"

#include <vector>

class LightExperiment : public Experiment {
  public:
    LightExperiment(Vector3D light_pos, float light_radius);
    ~LightExperiment();

    ExperimentType getType(void) const;

    std::vector<float> getConditionalResultProbabilities(unsigned num_possible_results, ObjectLegoModel model);

    ExperimentResult performExperiment(void);
    ExperimentResult performVirtualExperiment(ObjectLegoModel model, bool render);

    bool canPerformExperiment(Transform arm_to_obj);

    void print(void);

    void save(std::ostream &out_stream);
    void load(std::istream &in_stream);

    Vector3D getLightPos(void){ return light_pos; }
    float getLightRadius(void){ return light_radius; }

  private:
    unsigned id;

    Vector3D light_pos;
    float light_radius;

    void addResult(ExperimentResult result, std::vector<float> &distribution);

    Vector3D getNoisyLightPos(Vector3D base_light_pos);
    float getNoisyLightRadius(float base_light_radius);
};


#endif
