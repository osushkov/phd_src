
#include "SpacioTemporalLocality.h"


SpacioTemporalLocality::SpacioTemporalLocality(){

}

SpacioTemporalLocality::~SpacioTemporalLocality(){

}

void SpacioTemporalLocality::addFeature(const StereoFeature &sfeature, FeatureObjectType type, 
                                        unsigned frame){
    LocalityFeature new_feature;
    new_feature.sfeature = sfeature;
    new_feature.type = type;
    new_feature.frame = frame;
    locality_features.push_back(new_feature);

    purgeExpiredFeatures(frame);
}

FeatureObjectType SpacioTemporalLocality::mostProbableType(Vector3D position, unsigned frame){
    std::list<LocalityFeature>::iterator it = locality_features.begin();
    float aweight = 0.0f, bweight = 0.0f;
    float sigma = 3.0f;
     
    while(it != locality_features.end()){
        float d = Common::vectorDist(it->sfeature.position, position);
        if(it->type == ARM_FEATURE){
            aweight += expf(-(d*d)/(2.0f*sigma*sigma));
        }
        else if(it->type == BACKGROUND_FEATURE){
            bweight += expf(-(d*d)/(2.0f*sigma*sigma));
        }

        ++it;
    }

    if(aweight > bweight){ return ARM_FEATURE; }
    else{ return BACKGROUND_FEATURE; }
}

void SpacioTemporalLocality::purgeExpiredFeatures(unsigned cur_frame){
    if(cur_frame < 3){ return; }
    while(locality_features.front().frame < cur_frame-3){
        locality_features.pop_front();
    }
}

