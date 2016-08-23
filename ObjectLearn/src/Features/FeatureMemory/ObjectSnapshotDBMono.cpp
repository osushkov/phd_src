/*
 * ObjectSnapshotDBMono.cpp
 *
 *  Created on: 26/08/2009
 *      Author: osushkov
 */


#include "ObjectSnapshotDBMono.h"
#include "../StereoFeature.h"
#include "../../Util/ParallelServer.h"

#include <fstream>

struct ObjectSnapshotDBMonoWorkerData {
    ObjectSnapshotDBMonoWorkerData(ObjectSnapshotMono *object_snapshot,
                                   const std::vector<feature> &scene_features,
                                   std::string object_hint) :
                                       object_snapshot(object_snapshot),
                                       scene_features(scene_features),
                                       object_hint(object_hint) {}


    ObjectSnapshotMono *object_snapshot;
    const std::vector<feature> &scene_features;
    std::vector<unsigned> matched_indexes;
    std::string object_hint;
};

class ObjectSnapshotDBMonoWorker : public ParallelExecutor {
  public:

    ObjectSnapshotDBMonoWorker(){};
    ~ObjectSnapshotDBMonoWorker(){};

    void performTask(void *task_data, unsigned rank, unsigned size){
        ObjectSnapshotDBMonoWorkerData *data = (ObjectSnapshotDBMonoWorkerData *)task_data;

        if(data->object_hint.size() == 0 || data->object_snapshot->getName() == data->object_hint){
            data->object_snapshot->matchScene(data->scene_features, data->matched_indexes);
        }
    }
};


float ObjectSnapshotDBMono::getSIFTMatchScore(float sift_distance){
    return 1.0f/(1.0f + exp((sift_distance-400.0f)/50.0f));

    /*
    static bool loaded;
    static float scores[350];

    if(!loaded){
        std::fstream scores_file("sift_match_scores.dat", std::ios::in);

        unsigned i = 0;
        while(!scores_file.eof()){
            float ref, score;
            scores_file >> ref;
            scores_file >> score;

            scores[i] = score;
            i++;
        }

        loaded = true;
    }

    unsigned index = (unsigned)(sift_distance/2.0f);
    if(index >= 350){ index = 349; }

    return scores[index];
    */
}


ObjectSnapshotDBMono::ObjectSnapshotDBMono(){
    cur_snapshot_id = 0;

    param_max_pixel_match_dist = 7.0f;
    param_min_pixel_sift_match = 0.5f;
    param_min_basis_pmatch = 0.75f;
    param_distance_score_sd = 0.2f;
    param_orientation_score_sd = 0.2f;

}

ObjectSnapshotDBMono::~ObjectSnapshotDBMono(){
    for(unsigned i = 0; i < object_snapshots.size(); i++){
        delete object_snapshots[i];
    }
}

void ObjectSnapshotDBMono::setParams(const std::vector<float> &params){
    assert(params.size() == 5);

    param_max_pixel_match_dist = params[0];
    param_min_pixel_sift_match = params[1];
    param_min_basis_pmatch = params[2];
    param_distance_score_sd = params[3];
    param_orientation_score_sd = params[4];
}

void ObjectSnapshotDBMono::load(std::string filename){
    std::fstream db_file(filename.c_str(), std::ios::in | std::ios::binary);

    unsigned num_snapshots;
    db_file.read((char*)&num_snapshots, sizeof(unsigned));

    for(unsigned i = 0; i < num_snapshots; i++){
        unsigned name_size;
        db_file.read((char*)&name_size, sizeof(unsigned));

        char* name_buffer = new char[name_size];
        db_file.read(name_buffer, name_size * sizeof(char));

        std::string object_name(name_buffer);

        ObjectSnapshotMono *new_snapshot=
            new ObjectSnapshotMono(object_name, cur_snapshot_id++, param_max_pixel_match_dist,
                                   param_min_pixel_sift_match, param_min_basis_pmatch,
                                   param_distance_score_sd, param_orientation_score_sd);

        new_snapshot->load(db_file);
        object_snapshots.push_back(new_snapshot);

        delete[] name_buffer;
    }
}

void ObjectSnapshotDBMono::save(std::string filename){
    std::fstream db_file(filename.c_str(), std::ios::out | std::ios::binary);

    unsigned num_snapshots = object_snapshots.size();
    db_file.write((char*)&num_snapshots, sizeof(unsigned));

    for(unsigned i = 0; i < object_snapshots.size(); i++){
        unsigned name_size = object_snapshots[i]->getName().size() + 1;
        db_file.write((char*)&name_size, sizeof(unsigned));

        db_file.write(object_snapshots[i]->getName().c_str(), name_size*sizeof(char));
        object_snapshots[i]->save(db_file);
    }
}

std::set<unsigned> ObjectSnapshotDBMono::matchScene(const std::vector<feature> &scene_features,
                                                    std::string object){

    float best_match_score = 0.0f;
    unsigned best_snapshot;
    std::set<unsigned> result;

    std::vector<ObjectSnapshotDBMonoWorkerData*> worker_data;
    for(unsigned i = 0; i < object_snapshots.size(); i++){
        worker_data.push_back(new ObjectSnapshotDBMonoWorkerData(object_snapshots[i],
                                                                 scene_features,
                                                                 object));
    }

    ParallelServer pserver(PSM_PIPELINE);
    ObjectSnapshotDBMonoWorker worker_thread;
    Util::Semaphore task_sem;

    for(unsigned i = 0; i < worker_data.size(); i++){
        ParallelTask task(&worker_thread, worker_data[i], &task_sem);
        pserver.executeParallelTask(task);
    }

    for(unsigned i = 0; i < worker_data.size(); i++){
        task_sem.wait();
    }

    for(unsigned i = 0; i < worker_data.size(); i++){
        if(worker_data[i]->matched_indexes.size() > 7){
            for(unsigned j = 0; j < worker_data[i]->matched_indexes.size(); j++){
                result.insert(worker_data[i]->matched_indexes[j]);
            }
        }
        delete worker_data[i];
    }

    return result;
}

void ObjectSnapshotDBMono::addSnapshot(std::string object_name,
                                       const std::vector<feature> &features){
    ObjectSnapshotMono *new_snapshot =
        new ObjectSnapshotMono(object_name, cur_snapshot_id++, param_max_pixel_match_dist,
                               param_min_pixel_sift_match, param_min_basis_pmatch,
                               param_distance_score_sd, param_orientation_score_sd);

    new_snapshot->addFeatures(features);
    object_snapshots.push_back(new_snapshot);
}

ObjectSnapshotMono* ObjectSnapshotDBMono::getSnapshotById(unsigned id){
    for(unsigned i = 0; i < object_snapshots.size(); i++){
        if(object_snapshots[i]->getId() == id){
            return object_snapshots[i];
        }
    }
    return NULL;
}

unsigned ObjectSnapshotDBMono::getNumFeatures(void){
    unsigned result = 0;
    for(unsigned i = 0; i < object_snapshots.size(); i++){
        result += object_snapshots[i]->getFeatures().size();
    }
    return result;
}
