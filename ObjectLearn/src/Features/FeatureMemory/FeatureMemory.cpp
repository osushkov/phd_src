

#include "FeatureMemory.h"
#include "../../Util/Common.h"

#include <string>
#include <fstream>

#define MAX_FEATURE_MATCH_DIFF 240.0f

/* the maximum number of keypoint NN candidates to check during BBF search */
#define KDTREE_BBF_MAX_NN_CHKS 5000

/* threshold on squared ratio of distances between NN and 2nd NN */
#define NN_SQ_DIST_RATIO_THR 0.49

/*
static bool match(const feature &sift_feature1, const feature &sift_feature2){
    float dist_sq = 0.0f;
    for(unsigned i = 0; i < 128; i++){
        float diff = (sift_feature1.descr[i]-sift_feature2.descr[i]);
        dist_sq += diff*diff;
        if(dist_sq > MAX_FEATURE_MATCH_DIFF*MAX_FEATURE_MATCH_DIFF){ return false; }
    }

    return true;
}
*/

FeatureMemory::FeatureMemory() {
    num_arm_features = 0;
    num_background_features = 0;

    all_features = NULL;
    background_features = NULL;

    root_all = NULL;
    root_background = NULL;
}

FeatureMemory::~FeatureMemory(){
    if(root_all != NULL){
        kdtree_release(root_all);
    }

    if(root_background != NULL){
        kdtree_release(root_background);
    }

    if(all_features != NULL){
        delete[] all_features;
    }

    if(background_features != NULL){
        delete[] background_features;
    }
}


void FeatureMemory::insertFeature(FeatureObjectType type, const feature &sift_feature,
                                  std::string object_name){
    FeatureMemoryElement new_element;
    new_element.type = type;
    new_element.sift_feature = sift_feature;
    new_element.sift_feature.category = (int)type;
    new_element.object_name = object_name;

    if(type == BACKGROUND_FEATURE){
        num_background_features++;
    }
    if(type == ARM_FEATURE){
        num_arm_features++;
    }

    all_memory_features.push_back(new_element);
}

FeatureObjectType FeatureMemory::classifyFeature(const feature &sift_feature,
                                                 FeatureMatchMode mode,
                                                 std::string &object_name) {

    if(mode == EUCLIDEAN_DISTANCE){
        float closest_dist = FLT_MAX;
        unsigned closest_index = 0;

        for(unsigned i = 0; i < all_memory_features.size(); i++){
            float dist = sqrtf(descr_dist_sq(&sift_feature, &all_memory_features[i].sift_feature));
            if(dist < closest_dist){
                closest_dist = dist;
                closest_index = i;
            }
        }

        if(closest_dist < MAX_FEATURE_MATCH_DIFF){
           if(all_memory_features[closest_index].type == OBJECT_FEATURE){
               object_name = all_memory_features[closest_index].object_name;
               return OBJECT_FEATURE;
           }
           else{
               return all_memory_features[closest_index].type;
           }
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
        struct feature **nbrs1, **nbrs2;
        float d0, d1;
        feature feat = sift_feature;
        kdtree_bbf_knn(root_all, &feat, 1, &nbrs1, KDTREE_BBF_MAX_NN_CHKS);

        if(nbrs1[0]->category == BACKGROUND_FEATURE){
            free(nbrs1);
            return BACKGROUND_FEATURE;
        }

        if(nbrs1[0]->category == ARM_FEATURE){
            free(nbrs1);
            return ARM_FEATURE;
        }

        kdtree_bbf_knn(root_background, &feat, 1, &nbrs2, KDTREE_BBF_MAX_NN_CHKS);

        d0 = sqrt(descr_dist_sq(&feat, nbrs1[0])); // distance to arm feature
        d1 = sqrt(descr_dist_sq(&feat, nbrs2[0])); // distance to background feature
        free(nbrs1);
        free(nbrs2);

        if(d0 < d1*0.8f){
            return OBJECT_FEATURE;
        }
    }

    return BACKGROUND_FEATURE;
}
/*
std::vector<FeatureObjectType>
FeatureMemory::classifyFeatures(const std::vector<feature> &sift_features, FeatureMatchMode mode){

    std::vector<FeatureObjectType> result;

    for(unsigned i = 0; i < sift_features.size(); i++){
        result.push_back(classifyFeature(sift_features[i], mode));
    }

    return result;
}
*/

std::vector<float> FeatureMemory::weightedClassification(const feature &sift_feature){
    std::vector<float> result((int)NUM_FEATURES, 0.0f);
    float weight_sum = 0.0f;

    for(unsigned i = 0 ; i < all_memory_features.size(); i++){
        float dist = sqrtf(descr_dist_sq(&sift_feature, &all_memory_features[i].sift_feature, 260.0f*260.0f));
        if(dist < 260.0f){
            dist = std::max<float>(0.0f, dist-200.0f);
            float weight = 1.0f/(dist + 1.0f);
            weight_sum += weight;
            result[(int)all_memory_features[i].type] += weight;
        }
    }

    if(weight_sum > 0.0f){
    for(unsigned i = 0; i < result.size(); i++){
        //result[i] /= weight_sum;
    }
    }

    return result;
}

void FeatureMemory::load(std::string filename){
    std::fstream mem_file(filename.c_str(), std::ios::in | std::ios::binary);
    unsigned mem_size = 0;
    mem_file.read((char*)&mem_size, sizeof(unsigned));

    for(unsigned i = 0; i < mem_size; i++){
        std::string object_name;

        unsigned type;
        mem_file.read((char*)&type, sizeof(unsigned));

        if(type == ARM_FEATURE){
            num_arm_features++;
        }
        else if(type == BACKGROUND_FEATURE){
            num_background_features++;
        }
        else if(type == OBJECT_FEATURE){
            unsigned length = 0;
            mem_file.read((char*)&length, sizeof(unsigned));

            char *tmp_buf = new char[length];
            mem_file.read(tmp_buf, length);
            object_name = std::string(tmp_buf);
            delete[] tmp_buf;
        }


        feature new_sift_feature;
        read_feature(mem_file, new_sift_feature);

        insertFeature((FeatureObjectType)type, new_sift_feature, object_name);
    }

    createKDTree();

    /*
    std::cout << all_memory_features.size() << " features loaded into memory." << std::endl;
    std::cout << num_arm_features << " arm features." << std::endl;
    std::cout << num_background_features << " background features." << std::endl;*/
}


void FeatureMemory::save(std::string filename){
    std::fstream mem_file(filename.c_str(), std::ios::out | std::ios::binary);
    unsigned mem_size = all_memory_features.size();
    mem_file.write((char*)&mem_size, sizeof(unsigned));

    for(unsigned i = 0; i < all_memory_features.size(); i++){
        unsigned type = (unsigned)(all_memory_features[i].type);
        mem_file.write((char*)&type, sizeof(unsigned));

        if(all_memory_features[i].type == OBJECT_FEATURE){
            unsigned length = all_memory_features[i].object_name.size() + 1;
            mem_file.write((char*)&length, sizeof(unsigned));
            mem_file.write(all_memory_features[i].object_name.c_str(), length);
        }

        write_feature(mem_file, all_memory_features[i].sift_feature);
    }

    printFeaturesComposition();
}

void FeatureMemory::rebuild(void){
    if(root_all != NULL){
        kdtree_release(root_all);
    }

    if(root_background != NULL){
        kdtree_release(root_background);
    }

    if(all_features != NULL){
        delete[] all_features;
    }

    if(background_features != NULL){
        delete[] background_features;
    }

    createKDTree();
}

void FeatureMemory::createKDTree(void){
    all_features = new feature[all_memory_features.size()];
    background_features = new feature[num_background_features+num_arm_features];

    unsigned a = 0, b = 0;
    for(unsigned i = 0; i < all_memory_features.size(); i++){
        if(all_memory_features[i].type != OBJECT_FEATURE){
            background_features[b] = all_memory_features[i].sift_feature;
            b++;
        }
        all_features[a] = all_memory_features[i].sift_feature;
        a++;
    }

    //assert(a == all_memory_features.size());
    //assert(b == num_background_features);

    root_all = kdtree_build(all_features, a);
    root_background = kdtree_build(background_features, b);
}

float FeatureMemory::findNearestNeighbour(const feature &sift_feature,
                                          FeatureMemoryElement &nearest_element){
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

float FeatureMemory::findNearestNeighbourExclusive(const feature &sift_feature,
                                                   FeatureMemoryElement &nearest_element,
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

void FeatureMemory::printFeaturesComposition(void){
    unsigned num_features[NUM_FEATURES] = {0};

    for(unsigned i = 0; i < all_memory_features.size(); i++){
        num_features[all_memory_features[i].type]++;
    }

    for(unsigned i = 0; i < NUM_FEATURES; i++){
        std::cout << "feature type(" << i << ") : " << num_features[i] << std::endl;
    }
}
