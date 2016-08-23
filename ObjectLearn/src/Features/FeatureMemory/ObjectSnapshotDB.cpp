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
    std::set<unsigned> matched_indexes;
    bool hi_accuracy;
};

class ObjectSnapshotDBWorker : public ParallelExecutor {
  public:

    ObjectSnapshotDBWorker(){};
    ~ObjectSnapshotDBWorker(){};

    void performTask(void *task_data, unsigned rank, unsigned size){
        ObjectSnapshotDBWorkerData *data = (ObjectSnapshotDBWorkerData *)task_data;
        data->object_snapshot.matchScene(data->scene_features, data->matched_indexes, data->hi_accuracy);

    }
};


// TODO: this is crap implementation.
float ObjectSnapshotDB::getSIFTMatchScore(float sift_distance){
    return 1.0f/(1.0f + exp((sift_distance-400.0f)/50.0f));
}

ObjectSnapshotDB* ObjectSnapshotDB::getArmSnapshotDB(void){
    static ObjectSnapshotDB* arm_snapshot_db;

    if(arm_snapshot_db == NULL){
        std::cout << "Loading arm snapshot DB" << std::endl;
        arm_snapshot_db = new ObjectSnapshotDB("arm");

        bool success = true;

        success &= arm_snapshot_db->load("arm_snapshots.dat");
        success &= arm_snapshot_db->load("arm_snapshots2.dat");
        success &= arm_snapshot_db->load("arm_snapshots_full.dat");
        success &= arm_snapshot_db->load("arm_snapshots_old.dat");

/*
        success &= arm_snapshot_db->load("arm_snapshots_flat0.dat");
        success &= arm_snapshot_db->load("arm_snapshots_flat1.dat");
*/
        if(!success){ // TODO: read the pathname from settings.
            std::cout << "Failed loading arm snapshot" << std::endl;
            delete arm_snapshot_db;
            arm_snapshot_db = NULL;
        }
        std::cout << "Arm snapshots: " << arm_snapshot_db->object_snapshots.size() << std::endl;
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

    const unsigned min_snapshot_matches = 5;
    for(unsigned i = 0; i < worker_data.size(); i++){
        if(worker_data[i]->matched_indexes.size() >= min_snapshot_matches){
            result.matched_scene_features.insert(worker_data[i]->matched_indexes.begin(),
                                                 worker_data[i]->matched_indexes.end());
            /*
            for(unsigned j = 0; j < worker_data[i]->matched_indexes.size(); j++){
                result.matched_scene_features.insert(worker_data[i]->matched_indexes[j].index0);
            }
            */
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
