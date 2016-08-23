
#include "ParallelServer.h"
#include <pthread.h>
#include <cassert>
#include <iostream>

#define DEFAULT_NUM_THREADS 4
#define MAX_OUTSTANDING_TASKS 16

ParallelServer::ParallelServer(ParallelServerMode mode) :
    mode(mode), num_threads(DEFAULT_NUM_THREADS) {

    initialise();
}

ParallelServer::ParallelServer(ParallelServerMode mode, unsigned num_threads) :
    mode(mode), num_threads(num_threads) {

    assert(num_threads > 0);
    initialise();
}

ParallelServer::~ParallelServer(){
    {
        boost::recursive_mutex::scoped_lock lock(slock);

        // Make sure that all worker thread task lists are empty.
        for(unsigned i = 0; i < num_threads; i++){
            assert(worker_tasks.at(i).size() == 0);
        }

        // Set the shutdown flag to true, and notify all worker threads
        // that they should wake up and shutdown.
        shutdown = true;
        waiting_for_work.notify_all();
    }

    // Wait for all of the workers to die.
    for(unsigned i = 0; i < num_threads; i++){
        all_workers_shutdown.wait();
    }

    // Cleaup all the data of the server.
    delete[] worker_threads;
    delete[] all_args;

    for(unsigned i = 0; i < worker_tasks.size(); i++){
        while(worker_tasks[i].size() > 0){
            delete worker_tasks[i].front();
            worker_tasks[i].pop_front();
        }
    }

    worker_threads = NULL;
    all_args = NULL;
}

void ParallelServer::executeParallelTask(ParallelTask task){
    task.cur_task_completeness = 0;

    {
        boost::recursive_mutex::scoped_lock lock(slock);

        while(num_outstanding_tasks > MAX_OUTSTANDING_TASKS){
            too_much_work.wait(lock);
        }

        ParallelTask *new_task = new ParallelTask(task);

        // If the server is working in pipeline mode, only put the task onto the queue of
        // one of the worker threads. If working in parallel mode, put it onto both worker
        // task queues.
        if(mode == PSM_PIPELINE){
            worker_tasks.at(cur_worker_insert).push_back(new_task);
            cur_worker_insert = (cur_worker_insert + 1)%worker_tasks.size();
        }
        else{
            for(unsigned i = 0; i < num_threads; i++){
                worker_tasks.at(i).push_back(new_task);
            }
        }

        num_outstanding_tasks++;
        waiting_for_work.notify_all(); // Tell the worker threads that they have new work to do.
    }
}

void* ParallelServer::worker(void *thread_args){
    assert(thread_args != NULL);

    WorkerArgs *args = (WorkerArgs *)thread_args;
    assert(args->server != NULL);
    assert(args->rank < args->server->num_threads);

    bool just_completed_task = false;
    ParallelTask *cur_task = NULL;

    while(true){

        // Wait for some work to come in.
        {
            boost::recursive_mutex::scoped_lock lock(args->server->slock);

            if(just_completed_task){
                // Once the task is done, increment cur_task_completeness, and if its fully complete,
                // signal to the task condition variable that it is, and remove the task from the top of the
                // task list.
                cur_task->cur_task_completeness++;

                // If in pipeline mode, once a worker thread finishes a task, that task is done, as we can
                // signal the waiting user code that we finished.
                // If in parallel mode, then we must wait until all worker threads have finished their
                // part of the task, and so we wait until the task completeness is equal to the number
                // of total worker threads before signalling the user code.
                if(args->server->mode == PSM_PIPELINE){
                    cur_task->signal_condition->signal();
                    args->server->num_outstanding_tasks--;

                    if(args->server->num_outstanding_tasks <= MAX_OUTSTANDING_TASKS){
                        args->server->too_much_work.notify_all();
                    }

                    delete cur_task;
                }
                else if(cur_task->cur_task_completeness == args->server->num_threads){
                    cur_task->signal_condition->signal();
                    args->server->num_outstanding_tasks--;

                    if(args->server->num_outstanding_tasks <= MAX_OUTSTANDING_TASKS){
                        args->server->too_much_work.notify_all();
                    }

                    delete cur_task;
                }
            }

            // Wait for work to come in. This is in a while loop instead of an if statement to account for the
            // fact that in pipeline mode, all worker threads are signalled when new work arrives, but only one
            // actually gets the task in their queue.
            while(args->server->worker_tasks.at(args->rank).size() == 0 && !args->server->shutdown){
                args->server->waiting_for_work.wait(lock);
            }

            if(args->server->shutdown){
                break;
            }

            assert(args->server->worker_tasks.at(args->rank).size() > 0);

            // Grab the oldest task still uncompleted.
            cur_task = args->server->worker_tasks.at(args->rank).front();
            args->server->worker_tasks.at(args->rank).pop_front();
        }

        cur_task->executor->performTask(cur_task->data, args->rank, args->server->num_threads);
        just_completed_task = true;
    }


    // Signal to the controlling thread that this worker thread is now shutdown successfully.
    args->server->all_workers_shutdown.signal();
    pthread_exit((void *)NULL);
}

void ParallelServer::initialise(void){
    pthread_attr_t attr;
    pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    num_outstanding_tasks = 0;
    cur_worker_insert = 0;
    shutdown = false;

    worker_tasks.resize(num_threads);
    worker_threads = new pthread_t[num_threads];
    all_args = new WorkerArgs[num_threads];
    for(unsigned i = 0; i < num_threads; i++){
        all_args[i].server = this;
        all_args[i].rank = i;

        pthread_create(&(worker_threads[i]), &attr, &ParallelServer::worker, (void *)&(all_args[i]));
    }
}

