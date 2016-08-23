

#ifndef _FeatureMemory_H_
#define _FeatureMemory_H_

#include <vector>
#include <string>

#include "../../Util/Common.h"
#include "../SIFT/imgfeatures.h"
#include "kdtree.h"


enum FeatureObjectType {
    ARM_FEATURE,
    BACKGROUND_FEATURE,
    OBJECT_FEATURE,
    NUM_FEATURES,
};

struct FeatureMemoryElement {
    FeatureObjectType type;
    feature sift_feature;
    std::string object_name;
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

    void insertFeature(FeatureObjectType type, const feature &sift_feature, std::string name = "");

    FeatureObjectType classifyFeature(const feature &sift_feature,
                                      FeatureMatchMode mode,
                                      std::string &object_name);

    std::vector<float> weightedClassification(const feature &sift_feature);

    //std::vector<FeatureObjectType> classifyFeatures(const std::vector<feature> &sift_features,
    //                                                FeatureMatchMode mode);

    void load(std::string filename);
    void save(std::string filename);

    void rebuild(void);

    unsigned getNumFeatures(void){
        return all_memory_features.size();
    }

  private:

    std::vector<FeatureMemoryElement> all_memory_features;

    unsigned num_arm_features, num_background_features;

    struct kd_node *root_all, *root_background;
    struct feature *all_features, *background_features;


    void createKDTree(void);

    float findNearestNeighbour(const feature &sift_feature,
                               FeatureMemoryElement &nearest_element);

    float findNearestNeighbourExclusive(const feature &sift_feature,
                                        FeatureMemoryElement &nearest_element,
                                        FeatureObjectType excluded_type);

    void printFeaturesComposition(void);
};


#endif

