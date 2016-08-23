/*
 * ObjectSnapshotDB.cpp
 *
 *  Created on: 11/08/2009
 *      Author: osushkov
 */

#include "ObjectSnapshotDB.h"
#include "../../Util/ParallelServer.h"
#include <fstream>

struct ObjectSnapshotDBWorkerData {
    ObjectSnapshotDBWorkerData(ObjectSnapshot &object_snapshot,
                               const std::vector<StereoFeature> &scene_features,
                               bool hi_accuracy) :
                                 object_snapshot(object_snapshot),
                                 scene_features(scene_features),
                                 hi_accuracy(hi_accuracy) {}


    ObjectSnapshot &object_snapshot;
    const std::vector<StereoFeature> &scene_features;
    std::vector<SnapshotMatchResult> matched_indexes;
    bool hi_accuracy;
};

class ObjectSnapshotDBWorker : public ParallelExecutor {
  public:

    ObjectSnapshotDBWorker(){};
    ~ObjectSnapshotDBWorker(){};

    void performTask(void *task_data, unsigned rank, unsigned size){
        ObjectSnapshotDBWorkerData *data = (ObjectSnapshotDBWorkerData *)task_data;

        std::vector<SnapshotMatchResult> m;
        if(data->object_snapshot.matchScene(data->scene_features, m, data->hi_accuracy) > 0.0f){
            data->matched_indexes = m;
        }
    }
};


// TODO: this is crap implementation.
float ObjectSnapshotDB::getSIFTMatchScore(float sift_distance){
    return 1.0f/(1.0f + exp((sift_distance-400.0f)/50.0f));
    /*
    static bool loaded
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

    return scores[index];*/
}

ObjectSnapshotDB* ObjectSnapshotDB::getArmSnapshotDB(void){
    static ObjectSnapshotDB* arm_snapshot_db;
    if(arm_snapshot_db == NULL){
        arm_snapshot_db = new ObjectSnapshotDB("arm");
        if(!arm_snapshot_db->load("arm_snapshots.dat")){ // TODO: read the pathname from settings.
            delete arm_snapshot_db;
            arm_snapshot_db = NULL;
        }
    }

    return arm_snapshot_db;
}


ObjectSnapshotDB::ObjectSnapshotDB(std::string name) :
    object_name(name), cur_snapshot_id(0) {
}

ObjectSnapshotDB::~ObjectSnapshotDB(){

}

bool ObjectSnapshotDB::load(std::string filename){
    std::fstream db_file(filename.c_str(), std::ios::in | std::ios::binary);

    if(db_file.eof() || db_file.bad() || !db_file.good()){
        return false;
    }

    unsigned num_snapshots;
    db_file.read((char*)&num_snapshots, sizeof(unsigned));

    for(unsigned i = 0; i < num_snapshots; i++){
        unsigned name_size;
        db_file.read((char*)&name_size, sizeof(unsigned));

        char* name_buffer = new char[name_size];
        db_file.read(name_buffer, name_size * sizeof(char));

        std::string object_name(name_buffer);

        ObjectSnapshot new_snapshot(object_name, cur_snapshot_id++);
        new_snapshot.load(db_file);
        assert(new_snapshot.getName() == object_name);
        object_snapshots.push_back(new_snapshot);

        delete[] name_buffer;
    }

    std::cout << "Loaded " << object_snapshots.size() << " snapshots." << std::endl;
    return true;
}

bool ObjectSnapshotDB::save(std::string filename){
    std::fstream db_file(filename.c_str(), std::ios::out | std::ios::binary);

    unsigned num_snapshots = object_snapshots.size();
    db_file.write((char*)&num_snapshots, sizeof(unsigned));

    for(unsigned i = 0; i < object_snapshots.size(); i++){
        unsigned name_size = object_snapshots[i].getName().size() + 1;
        db_file.write((char*)&name_size, sizeof(unsigned));

        db_file.write(object_snapshots[i].getName().c_str(), name_size*sizeof(char));
        object_snapshots[i].save(db_file);
    }

    return true;
}

SceneMatchResult ObjectSnapshotDB::matchScene(const std::vector<StereoFeature> &scene_features,
                                              bool hi_accuracy){
    std::vector<ObjectSnapshotDBWorkerData*> worker_data;
    for(unsigned i = 0; i < object_snapshots.size(); i++){
        worker_data.push_back(new ObjectSnapshotDBWorkerData(object_snapshots[i],
                                                             scene_features,
                                                             hi_accuracy));
    }

    ParallelServer pserver(PSM_PIPELINE);
    ObjectSnapshotDBWorker worker_thread;
    Util::Semaphore task_sem;

    for(unsigned i = 0; i < worker_data.size(); i++){
        ParallelTask task(&worker_thread, worker_data[i], &task_sem);
        pserver.executeParallelTask(task);
    }

    for(unsigned i = 0; i < worker_data.size(); i++){
        task_sem.wait();
    }

    SceneMatchResult result;

    const unsigned min_snapshot_matches = 3;
    for(unsigned i = 0; i < worker_data.size(); i++){
        if(worker_data[i]->matched_indexes.size() >= min_snapshot_matches){
            for(unsigned j = 0; j < worker_data[i]->matched_indexes.size(); j++){
                result.matched_scene_features.insert(worker_data[i]->matched_indexes[j].index0);
            }
        }
        delete worker_data[i];
    }

    return result;
}

void ObjectSnapshotDB::addSnapshot(const std::vector<StereoFeature> &features){
    ObjectSnapshot new_snapshot(object_name, cur_snapshot_id++);
    new_snapshot.addFeatures(features);
    object_snapshots.push_back(new_snapshot);
}

ObjectSnapshot* ObjectSnapshotDB::getSnapshotById(unsigned id){
    for(unsigned i = 0; i < object_snapshots.size(); i++){
        if(object_snapshots[i].getId() == id){
            return &(object_snapshots[i]);
        }
    }
    return NULL;
}
