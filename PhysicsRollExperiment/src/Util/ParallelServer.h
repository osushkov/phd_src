
//! TODO write a short tutorial on how to use ParallelServer
//! TODO get rid of pthreads, use boost threads instead.

#ifndef _ParallelServer_H_
#define _ParallelServer_H_

#include <boost/thread/condition.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include "Semaphore.h"

#include <list>
#include <vector>
#include <pthread.h>

enum ParallelServerMode {
    PSM_PIPELINE,
    PSM_PARALLEL
};

class ParallelExecutor {
  public:
    virtual ~ParallelExecutor(){};
    virtual void performTask(void *task_data, unsigned rank, unsigned size) = 0;

};

class ParallelTask {
  public:
    ParallelTask(ParallelExecutor *executor, void* data, Util::Semaphore *signal_condition) :
        executor(executor), data(data), signal_condition(signal_condition),
        cur_task_completeness(0) {}

    ParallelTask(const ParallelTask &task){
        executor = task.executor;
        data = task.data;
        signal_condition = task.signal_condition;
        cur_task_completeness = task.cur_task_completeness;
    }


    ParallelExecutor *executor;
    void* data;

    Util::Semaphore *signal_condition;
    unsigned cur_task_completeness;
};

class ParallelServer {

  public:

    ParallelServer(ParallelServerMode mode);
    ParallelServer(ParallelServerMode mode, unsigned num_threads);

    ~ParallelServer();

    void executeParallelTask(ParallelTask task);

  private:

    unsigned cur_worker_insert;
    const ParallelServerMode mode;
    const unsigned num_threads;
    pthread_t *worker_threads;

    std::list<ParallelTask> task_list;
    std::vector< std::list<ParallelTask *> > worker_tasks;

    boost::condition waiting_for_work;
    boost::recursive_mutex slock;

    boost::condition too_much_work;
    unsigned num_outstanding_tasks;

    struct WorkerArgs {
        ParallelServer *server;
        unsigned rank;
    };

    bool shutdown;
    Util::Semaphore all_workers_shutdown;

    WorkerArgs *all_args;

    static void* worker(void *thread_args);
    void initialise(void);
};


#endif

