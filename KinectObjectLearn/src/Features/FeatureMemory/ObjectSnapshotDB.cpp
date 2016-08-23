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
    ObjectSnapshotDBWorkerData(ObjectSnapshot *object_snapshot,
                               const std::vector<SIFTFeature3D> &scene_features,
                               bool hi_accuracy) :
                                 object_snapshot(object_snapshot),
                                 scene_features(scene_features),
                                 hi_accuracy(hi_accuracy) {}


    ObjectSnapshot *object_snapshot;
    const std::vector<SIFTFeature3D> &scene_features;
    SceneMatchResult match_result;
    bool hi_accuracy;
};

class ObjectSnapshotDBWorker : public ParallelExecutor {
  public:

    ObjectSnapshotDBWorker(){};
    ~ObjectSnapshotDBWorker(){};

    void performTask(void *task_data, unsigned rank, unsigned size){
        ObjectSnapshotDBWorkerData *data = (ObjectSnapshotDBWorkerData *)task_data;

        data->match_result.matched_snapshot = data->object_snapshot;   
        data->match_result.match_score = data->object_snapshot->matchScene(data->scene_features, data->match_result.feature_matches, 
                                                                           data->match_result.approximate_transform, data->hi_accuracy);
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

        success &= arm_snapshot_db->load("data/arm_snapshots.dat");

        if(!success){ // TODO: read the pathname from settings.
            std::cout << "Failed loading arm snapshot" << std::endl;
            delete arm_snapshot_db;
            arm_snapshot_db = NULL;
        }
        std::cout << "Arm snapshots: " << arm_snapshot_db->object_snapshots.size() << std::endl;
    }

    return arm_snapshot_db;
}

std::set<unsigned> ObjectSnapshotDB::getMatchedFeatureIndexes(const std::vector<SIFTFeature3D> &features,
                                                              const SceneMatchResult &match){
    std::set<unsigned> result;

    for(unsigned i = 0; i < match.feature_matches.size(); i++){
        result.insert(match.feature_matches[i].scene_index);
    }

    return result;
}

ObjectSnapshot* ObjectSnapshotDB::createNewSnapshot(std::string name){
    return new ObjectSnapshot(name, cur_snapshot_id++);
}

ObjectSnapshot* ObjectSnapshotDB::createNewSnapshot(std::string name, Vector3D to_camera){
    return new ObjectSnapshot(name, cur_snapshot_id++, to_camera);
}

ObjectSnapshotDB::ObjectSnapshotDB(std::string name) :
    object_name(name), cur_snapshot_id(0) {
}

ObjectSnapshotDB::~ObjectSnapshotDB(){
    /*
    for(unsigned i = 0; i < object_snapshots.size(); i++){
        delete object_snapshots[i];
    }
    */
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

        ObjectSnapshot *new_snapshot = new ObjectSnapshot(object_name, cur_snapshot_id++);
        new_snapshot->load(db_file);
        assert(new_snapshot->getName() == object_name);
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
        unsigned name_size = object_snapshots[i]->getName().size() + 1;
        db_file.write((char*)&name_size, sizeof(unsigned));

        db_file.write(object_snapshots[i]->getName().c_str(), name_size*sizeof(char));
        object_snapshots[i]->save(db_file);
    }

    return true;
}

SceneMatchResult ObjectSnapshotDB::matchScene(const std::vector<SIFTFeature3D> &scene_features,
                                              bool hi_accuracy){
    return matchSceneWithSnapshots(scene_features, object_snapshots, hi_accuracy);
}

SceneMatchResult ObjectSnapshotDB::matchScene(const std::vector<SIFTFeature3D> &scene_features,
                                              Vector3D view_vector, bool hi_accuracy){
    const float max_angle_diff = 20.0f*(float)M_PI/180.0f;
    std::vector<ObjectSnapshot*> snapshot_pool;

    for(unsigned i = 0; i < object_snapshots.size(); i++){
        if(fabs(acosf(object_snapshots[i]->getToCamera().dotProduct(view_vector))) <= max_angle_diff){
            snapshot_pool.push_back(object_snapshots[i]);
        }
    }

    return matchSceneWithSnapshots(scene_features, snapshot_pool, hi_accuracy);
}

std::vector<SceneMatchResult> ObjectSnapshotDB::matchSceneMany(const std::vector<SIFTFeature3D> &scene_features,
                                                               float match_threshold, 
                                                               bool hi_accuracy){
    return matchSceneWithManySnapshots(scene_features, object_snapshots, match_threshold, hi_accuracy);
}

std::vector<SceneMatchResult> ObjectSnapshotDB::matchSceneMany(const std::vector<SIFTFeature3D> &scene_features,
                                                               Vector3D view_vector, 
                                                               float match_threshold, 
                                                               bool hi_accuracy){
    const float max_angle_diff = 20.0f*(float)M_PI/180.0f;
    std::vector<ObjectSnapshot*> snapshot_pool;

    for(unsigned i = 0; i < object_snapshots.size(); i++){
        if(acosf(object_snapshots[i]->getToCamera().dotProduct(view_vector)) <= max_angle_diff){
            snapshot_pool.push_back(object_snapshots[i]);
        }
    }

    return matchSceneWithManySnapshots(scene_features, snapshot_pool, match_threshold, hi_accuracy);
}

void ObjectSnapshotDB::addSnapshot(ObjectSnapshot *snapshot){
    //ObjectSnapshot new_snapshot(object_name, cur_snapshot_id++);
    //new_snapshot.addFeatures(features, std::vector<SurfacePoint>());
    object_snapshots.push_back(snapshot);
}

ObjectSnapshot* ObjectSnapshotDB::getSnapshotById(unsigned id){
    for(unsigned i = 0; i < object_snapshots.size(); i++){
        if(object_snapshots[i]->getId() == id){
            return object_snapshots[i];
        }
    }
    return NULL;
}

SceneMatchResult ObjectSnapshotDB::matchSceneWithSnapshots(const std::vector<SIFTFeature3D> &scene_features,
                                                           std::vector<ObjectSnapshot*> snapshots_pool,
                                                           bool hi_accuracy){

    std::vector<ObjectSnapshotDBWorkerData*> worker_data;
    for(unsigned i = 0; i < snapshots_pool.size(); i++){
        worker_data.push_back(new ObjectSnapshotDBWorkerData(snapshots_pool[i],
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
    result.matched_snapshot = NULL;
    result.match_score = 0.0f;

    float best_match_score = 0.0f;
    for(unsigned i = 0; i < worker_data.size(); i++){
        if(worker_data[i]->match_result.match_score > best_match_score){
            best_match_score = worker_data[i]->match_result.match_score;
            result = worker_data[i]->match_result;
        }
    }

    for(unsigned i = 0; i < worker_data.size(); i++){
        delete worker_data[i];
    }

    return result;
}

std::vector<SceneMatchResult> ObjectSnapshotDB::matchSceneWithManySnapshots(const std::vector<SIFTFeature3D> &scene_features,
                                                                            std::vector<ObjectSnapshot*> snapshots_pool,
                                                                            float match_threshold,
                                                                            bool hi_accuracy){
    std::cout << "pool size: " << snapshots_pool.size() << std::endl;
    std::vector<ObjectSnapshotDBWorkerData*> worker_data;
    for(unsigned i = 0; i < snapshots_pool.size(); i++){
        worker_data.push_back(new ObjectSnapshotDBWorkerData(snapshots_pool[i],
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

    std::vector<SceneMatchResult> result;

    for(unsigned i = 0; i < worker_data.size(); i++){
        if(worker_data[i]->match_result.match_score >= match_threshold){
            result.push_back(worker_data[i]->match_result);
        }
    }

    for(unsigned i = 0; i < worker_data.size(); i++){
        delete worker_data[i];
    }

    std::cout << "result size: " << result.size() << std::endl;
    return result;
}