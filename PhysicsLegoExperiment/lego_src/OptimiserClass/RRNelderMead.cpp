
#include "RRNelderMead.h"
#include "NelderMead.h"
#include "../Util/ParallelServer.h"
#include "../Util/Semaphore.h"

#include <iostream>

struct RRNMData {
    RRNMData(OptimisableFunction *target, unsigned num_params,
             unsigned max_evals, float target_fitness) :
        target(target), num_params(num_params),
        max_evals(max_evals), target_fitness(target_fitness) {

    }

    OptimisableFunction *target;
    unsigned num_params;
    unsigned max_evals;
    float target_fitness;

    std::vector<float> best_params;
    float best_fitness;
};

class RRNMWorker : public ParallelExecutor {
  public:
    RRNMWorker(){};
    ~RRNMWorker(){};

    void performTask(void *task_data, unsigned rank, unsigned size){
        RRNMData *data = (RRNMData*)task_data;

        NelderMead optimiser;
        data->best_params = optimiser.optimise(data->target, data->num_params,
                                               data->max_evals, data->target_fitness);
        data->best_fitness = data->target->eval(data->best_params);
    }
};


RRNelderMead::RRNelderMead(unsigned num_workers) :
    num_workers(num_workers) {

}

RRNelderMead::~RRNelderMead(){

}

std::vector<float> RRNelderMead::optimise(OptimisableFunction *target, unsigned num_params,
                                          unsigned max_evals, float target_fitness){

    ParallelServer pserver(PSM_PIPELINE);

    std::vector<Util::Semaphore*> task_sem;
    std::vector<RRNMData*> all_data;
    std::vector<ParallelTask> tasks;
    RRNMWorker worker_thread;

    for(unsigned i = 0; i < num_workers; i++){
        Util::Semaphore *new_sem = new Util::Semaphore();
        task_sem.push_back(new_sem);

        RRNMData *new_data = new RRNMData(target, num_params, max_evals, target_fitness);
        all_data.push_back(new_data);

        ParallelTask new_task(&worker_thread, new_data, new_sem);
        tasks.push_back(new_task);

        pserver.executeParallelTask(tasks.back());
    }

    float best_fitness = 0.0f;
    std::vector<float> best_params;
    for(unsigned i = 0; i < task_sem.size(); i++){
        task_sem[i]->wait();
       
        if(i == 0 || all_data[i]->best_fitness < best_fitness){
            best_fitness = all_data[i]->best_fitness;
            best_params = all_data[i]->best_params;
        }

        delete task_sem[i];
        delete all_data[i];

    }

    return best_params;
}
