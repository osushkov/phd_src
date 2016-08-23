
#include "SpacioTemporalLocality.h"
#include <cmath>


SpacioTemporalLocality::SpacioTemporalLocality(){

}

SpacioTemporalLocality::~SpacioTemporalLocality(){

}

void SpacioTemporalLocality::addFeatures(const std::vector<StereoFeature> &sfeatures,
                                         const std::vector<FeatureObjectType> &types,
                                         unsigned frame){

    assert(sfeatures.size() == types.size());

    for(unsigned i = 0; i < sfeatures.size(); i++){
        LocalityFeature new_feature;
        new_feature.sfeature = sfeatures[i];
        new_feature.type = types[i];
        new_feature.frame = frame;
        locality_features.push_back(new_feature);
    }

    purgeExpiredFeatures(frame);
}

FeatureObjectType SpacioTemporalLocality::mostProbableType(Vector3D position, unsigned frame){
    std::list<LocalityFeature>::iterator it = locality_features.begin();
    float sigma = 3.0f;

    float weights[NUM_FEATURES] = {0.0f};

    while(it != locality_features.end()){
        float d = (it->sfeature.position - position).length();
        float frame_diff = frame - it->frame;
        float attenuation = powf(2.0f, frame_diff);

        weights[it->type] += attenuation*expf(-(d*d)/(2.0f*sigma*sigma));
        ++it;
    }

    weights[ARM_FEATURE] *= 1.2f;
    FeatureObjectType winning_type = BACKGROUND_FEATURE;
    float max_weight = 0.0f;
    for(unsigned i = 0; i < NUM_FEATURES; i++){
        if(weights[i] > max_weight){
            max_weight = weights[i];
            winning_type = (FeatureObjectType)i;
        }
    }
    return winning_type;
}

void SpacioTemporalLocality::purgeExpiredFeatures(unsigned cur_frame){
    if(cur_frame < 3){ return; }
    while(locality_features.front().frame < cur_frame-3){
        locality_features.pop_front();
    }
}

