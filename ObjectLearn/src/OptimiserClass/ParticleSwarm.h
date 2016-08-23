
#ifndef _ParticleSwarm_H_
#define _ParticleSwarm_H_

#include "Optimiser.h"
#include <vector>

#include "../Util/ParallelServer.h"
#include "../Util/Semaphore.h"


class ParticleSwarm : public Optimiser {
  public:
    ParticleSwarm(unsigned num_particles);
    ~ParticleSwarm();

    std::vector<float> optimise(OptimisableFunction *target, unsigned num_params,
                                unsigned max_evals, float target_fitness=0.0f);

  private:
    struct Particle {
        std::vector<float> position;
        std::vector<float> velocity;

        std::vector<float> best_known_position;
        float best_known_value;
    };

    std::vector<float> best_global_position;
    float best_global_value;
    unsigned last_lowest_iter;

    unsigned num_params, num_particles;

    std::vector<Particle> swarm;
    float omega, c1, c2;

    OptimisableFunction *func_target;

    ParallelServer pserver;
    Util::Semaphore lock;

    unsigned iter_limit, cur_iter;


    void updateParticles(OptimisableFunction *target);
    std::vector<float> createRandomVector(unsigned length, float min, float max);

    std::vector<float> componentMultiply(std::vector<float> var1, std::vector<float> var2);
    std::vector<float> scalarMultiply(float scal, std::vector<float> var);
    std::vector<float> componentAdd(std::vector<float> var1, std::vector<float> var2);

    Particle createInitialParticle(OptimisableFunction *target);

    friend class ParticleSwarmWorker;
};

#endif

