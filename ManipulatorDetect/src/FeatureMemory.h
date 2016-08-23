
#ifndef _FeatureMemory_H_
#define _FeatureMemory_H_

#include <vector>
#include <string>
#include <ANN/ANN.h>

#include "Util/Common.h"
#include "Util/ParallelServer.h"
#include "SIFT/imgfeatures.h"


enum FeatureObjectType {
    ARM_FEATURE,
    BACKGROUND_FEATURE,
    NUM_FEATURES,
};

struct FeatureMemoryElement {
    FeatureObjectType type;
    feature sift_feature;
};

enum FeatureMatchMode {
    EUCLIDEAN_DISTANCE,
    EXACT_NN,
    APPROX_NN,
};

//////////////////////////////////////////////////////////

class FeatureMemory {
  public:
    FeatureMemory();
    ~FeatureMemory();

    void insertFeature(FeatureObjectType type, const feature &sift_feature);

    FeatureObjectType classifyFeature(const feature &sift_feature, FeatureMatchMode mode);
    std::vector<FeatureObjectType> classifyFeatures(const std::vector<feature> &sift_features, FeatureMatchMode mode);

    void load(std::string filename);
    void save(std::string filename);

  private:

    std::vector<FeatureMemoryElement> all_memory_features;
    ParallelServer pserver; 

    unsigned num_arm_features, num_background_features;
    ANNpointArray data_points_all, data_points_background;
    ANNkd_tree* kd_tree_all, *kd_tree_background;   

    
    void createKDTree(void);
 
    float findNearestNeighbour(const feature &sift_feature, FeatureMemoryElement &nearest_element);
    float findNearestNeighbourExclusive(const feature &sift_feature, FeatureMemoryElement &nearest_element, 
                                        FeatureObjectType excluded_type);
};


#endif

