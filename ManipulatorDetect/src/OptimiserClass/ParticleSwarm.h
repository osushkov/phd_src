
#ifndef _ParticleSwarm_H_
#define _ParticleSwarm_H_

#include "Optimiser.h"
#include <vector>


class ParticleSwarm : public Optimiser {
  public:
    ParticleSwarm(unsigned num_particles);
    ~ParticleSwarm();

    std::vector<float> optimise(OptimisableFunction *target, unsigned num_params, 
                                unsigned max_evals, std::vector<float> dimension_magnitude);

  private:
    struct Particle {
        std::vector<float> position;
        std::vector<float> velocity;
        
        std::vector<float> best_known_position;
        float best_known_value;
    };


    std::vector<float> best_global_position;
    float best_global_value;

    unsigned num_params, num_particles;

    std::vector<Particle> swarm;
    float omega, c1, c2;

    void updateParticles(OptimisableFunction *target);
    std::vector<float> createRandomVector(unsigned length, float min, float max);

    std::vector<float> componentMultiply(std::vector<float> var1, std::vector<float> var2);
    std::vector<float> scalarMultiply(float scal, std::vector<float> var);
    std::vector<float> componentAdd(std::vector<float> var1, std::vector<float> var2);

    Particle createInitialParticle(OptimisableFunction *target);
};

#endif

