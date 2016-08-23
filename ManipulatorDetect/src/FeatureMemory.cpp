    

#include "FeatureMemory.h"
#include "Util/Common.h"
#include "Util/ParallelServer.h"

#include <string>
#include <fstream>

#define MAX_FEATURE_MATCH_DIFF 210.0f


static bool match(const feature &sift_feature1, const feature &sift_feature2){
    float dist_sq = 0.0f;
    for(unsigned i = 0; i < 128; i++){
        float diff = (sift_feature1.descr[i]-sift_feature2.descr[i]);
        dist_sq += diff*diff;
        if(dist_sq > MAX_FEATURE_MATCH_DIFF*MAX_FEATURE_MATCH_DIFF){ return false; }
    }

    return true;
}


struct FeatureMemoryWorkerData {
    FeatureMemoryWorkerData(FeatureMemory *feature_memory,
                            const std::vector<FeatureMemoryElement> &all_memory_features, 
                            const std::vector<feature> &qfeatures,
                            std::vector<FeatureObjectType> &result,
                            FeatureMatchMode mode) :
        feature_memory(feature_memory),
        all_memory_features(all_memory_features),
        qfeatures(qfeatures),
        result(result),
        mode(mode) {}

    FeatureMemory *feature_memory;
    const std::vector<FeatureMemoryElement> &all_memory_features;
    const std::vector<feature> &qfeatures;
    std::vector<FeatureObjectType> &result;
    FeatureMatchMode mode;
};


class FeatureMemoryWorker : public ParallelExecutor {
  public:

    FeatureMemoryWorker(){};
    ~FeatureMemoryWorker(){};

    void performTask(void *task_data, unsigned rank, unsigned size){
        FeatureMemoryWorkerData *data = (FeatureMemoryWorkerData *)task_data;
        assert(data != NULL);
        assert(data->result.size() == data->qfeatures.size());

        unsigned start = (rank*data->qfeatures.size())/size;
        unsigned end = ((rank+1)*data->qfeatures.size())/size;

        for(unsigned i = start; i < end; i++){
            data->result[i] = data->feature_memory->classifyFeature(data->qfeatures[i], data->mode);
        }
    }
};


FeatureMemory::FeatureMemory() : pserver(PSM_PARALLEL) {
    num_arm_features = 0;
    num_background_features = 0;
}

FeatureMemory::~FeatureMemory(){

}


void FeatureMemory::insertFeature(FeatureObjectType type, const feature &sift_feature){
    FeatureMemoryElement new_element;
    new_element.type = type;
    new_element.sift_feature = sift_feature;
    all_memory_features.push_back(new_element);
}


FeatureObjectType FeatureMemory::classifyFeature(const feature &sift_feature, FeatureMatchMode mode) {
    if(mode == EUCLIDEAN_DISTANCE){
        for(unsigned i = 0; i < all_memory_features.size(); i++){
            if(match(sift_feature, all_memory_features[i].sift_feature)){ return all_memory_features[i].type; }
        }
        return BACKGROUND_FEATURE;
    }
    else if(mode == EXACT_NN){/*
        FeatureMemoryElement nn, snn;
        nn.type = snn.type = BACKGROUND_FEATURE;

        float nearest_dist = findNearestNeighbour(feature, nn);
        if(nn.type == BACKGROUND_FEATURE){ return BACKGROUND_FEATURE; }
        float second_nearest_dist = findNearestNeighbourExclusive(feature, snn, nn.type);

        //std::cout << nearest_dist << " " << second_nearest_dist << std::endl;
        if(0.85f*second_nearest_dist < nearest_dist){
            return BACKGROUND_FEATURE;
        }
        else{
            return nn.type;
        }*/
    }
    else if(mode == APPROX_NN){
        ANNidxArray nn_index = new ANNidx[1];
        ANNdistArray dists = new ANNdist[1];	

        ANNpoint query_point = annAllocPt(128);
        for(unsigned i = 0; i < 128; i++){
            query_point[i] = sift_feature.descr[i];
        }

        kd_tree_all->annkSearch(query_point, 1, nn_index, dists, 2.0f);
        if(all_memory_features[nn_index[0]].type == BACKGROUND_FEATURE){ return BACKGROUND_FEATURE; }
        //if(dists[0] < MAX_FEATURE_MATCH_DIFF){ return ARM_FEATURE; }
 
        float nn_dist = dists[0];
        kd_tree_background->annkSearch(query_point, 1, nn_index, dists, 2.0f);

        if(nn_dist < 0.8f*dists[0]){ return ARM_FEATURE; }
       
        annDeallocPt(query_point); 
        delete[] nn_index;
        delete[] dists;
    }

    return BACKGROUND_FEATURE;
}

std::vector<FeatureObjectType> FeatureMemory::classifyFeatures(const std::vector<feature> &sift_features, FeatureMatchMode mode){
    std::vector<FeatureObjectType> result;

    for(unsigned i = 0; i < sift_features.size(); i++){
        result.push_back(classifyFeature(sift_features[i], mode));
    }

    return result;


    // TODO: when I have a non-retarded multi-threaded library, get that to work
/*
    FeatureMemoryWorker worker_thread;
    Util::Semaphore task_sem;
    
    std::vector<FeatureObjectType> result(sift_features.size());
    FeatureMemoryWorkerData data(this, all_memory_features, sift_features, result, mode);
    ParallelTask task(&worker_thread, &data, &task_sem);
    pserver.executeParallelTask(task);
    task_sem.wait();
*/
    return result;
}


void FeatureMemory::load(std::string filename){
    std::fstream mem_file(filename.c_str(), std::ios::in | std::ios::binary);
    unsigned mem_size = 0;
    mem_file.read((char*)&mem_size, sizeof(unsigned));

    for(unsigned i = 0; i < mem_size; i++){
        unsigned type;
        mem_file.read((char*)&type, sizeof(unsigned));

        if(type == ARM_FEATURE){ num_arm_features++; }
        if(type == BACKGROUND_FEATURE){ num_background_features++; }

        feature new_sift_feature;
        read_feature(mem_file, new_sift_feature);
    
        insertFeature((FeatureObjectType)type, new_sift_feature);
    }

    createKDTree();

    std::cout << all_memory_features.size() << " features loaded into memory." << std::endl;
    std::cout << num_arm_features << " arm features." << std::endl;
    std::cout << num_background_features << " background features." << std::endl;
}


void FeatureMemory::save(std::string filename){
    std::fstream mem_file(filename.c_str(), std::ios::out | std::ios::binary);
    unsigned mem_size = all_memory_features.size();
    mem_file.write((char*)&mem_size, sizeof(unsigned));

    for(unsigned i = 0; i < all_memory_features.size(); i++){
        unsigned type = (unsigned)(all_memory_features[i].type);
        mem_file.write((char*)&type, sizeof(unsigned));

        write_feature(mem_file, all_memory_features[i].sift_feature);
    }
}

void FeatureMemory::createKDTree(void){
    data_points_all = annAllocPts(all_memory_features.size(), 128);
    data_points_background = annAllocPts(num_background_features, 128);

    unsigned b = 0;
    for(unsigned i = 0; i < all_memory_features.size(); i++){
        for(unsigned j = 0; j < 128; j++){
            data_points_all[i][j] = all_memory_features[i].sift_feature.descr[j];
        }

        if(all_memory_features[i].type == BACKGROUND_FEATURE){
            for(unsigned j = 0; j < 128; j++){
                data_points_background[b][j] = all_memory_features[i].sift_feature.descr[j];
            }
            b++;
        }
    }
    assert(b == num_background_features);

    kd_tree_all = new ANNkd_tree(data_points_all, all_memory_features.size(), 128);
    kd_tree_background = new ANNkd_tree(data_points_background, num_background_features, 128);
}

float FeatureMemory::findNearestNeighbour(const feature &sift_feature, FeatureMemoryElement &nearest_element){
    float nearest_dist = FLT_MAX;

    for(unsigned i = 0; i < all_memory_features.size(); i++){
        float dist = descr_dist_sq(&(all_memory_features[i].sift_feature), &sift_feature);
        if(dist < nearest_dist){
            nearest_dist = dist;
            nearest_element = all_memory_features[i];
        }
    }

    return nearest_dist;
}

float FeatureMemory::findNearestNeighbourExclusive(const feature &sift_feature, FeatureMemoryElement &nearest_element,
                                                   FeatureObjectType excluded_type){
    float nearest_dist = FLT_MAX;

    for(unsigned i = 0; i < all_memory_features.size(); i++){
        if(all_memory_features[i].type == excluded_type){ continue; }

        float dist = descr_dist_sq(&(all_memory_features[i].sift_feature), &sift_feature);
        if(dist < nearest_dist){
            nearest_dist = dist;
            nearest_element = all_memory_features[i];
        }
    }

    return nearest_dist;

}

