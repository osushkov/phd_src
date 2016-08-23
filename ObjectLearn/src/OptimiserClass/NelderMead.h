
#ifndef _NelderMead_H_
#define _NelderMean_H_

#include "Optimiser.h"
#include <vector>

class NelderMead : public Optimiser {
  public:
    NelderMead();
    NelderMead(std::vector<float> start_vals);
    ~NelderMead();

    std::vector<float> optimise(OptimisableFunction *target, unsigned num_params,
                                unsigned max_evals, float target_fitness=0.0f);

  private:

    struct SimplexVertex {
        std::vector<float> coord;
        float value;
    };

    struct SVComp {
        bool operator()(const SimplexVertex &v1, const SimplexVertex &v2){
            return v1.value < v2.value;
        }
    };

    unsigned iter_limit, cur_iters;

    unsigned num_params;
    std::vector<SimplexVertex> simplex_vertex; // all of the vertices of the simpex. should be n+1 vertices
    std::vector<float> start_vals;

    std::vector<float> centreOfMass(const std::vector<SimplexVertex> &vertices,
                                    unsigned num_vertices);

    std::vector<SimplexVertex> createInitialSimplex(OptimisableFunction *target, unsigned dim);

    SimplexVertex reflectedVertex(OptimisableFunction *target, SimplexVertex vertex,
                                  std::vector<float> rpoint, float reflect_dist=1.0f);

    SimplexVertex contractedVertex(OptimisableFunction *target, SimplexVertex vertex,
                                   std::vector<float> cpoint, float contract_ratio=0.5f);

};

void setPrint(bool var);

#endif

