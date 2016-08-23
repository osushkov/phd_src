
#ifndef _SpacioTemporalLocality_H_
#define _SpacioTemporalLocality_H_

#include <list>

#include "Util/Common.h"
#include "StereoFeatureCorrelation.h"
#include "FeatureMemory.h"


struct LocalityFeature {
    StereoFeature sfeature;
    FeatureObjectType type;
    unsigned frame;
};

class SpacioTemporalLocality {
  public:
    SpacioTemporalLocality();
    ~SpacioTemporalLocality();

    void addFeature(const StereoFeature &sfeature, FeatureObjectType type, unsigned frame); 
    FeatureObjectType mostProbableType(Vector3D position, unsigned frame);

  private:

    std::list<LocalityFeature> locality_features;

    void purgeExpiredFeatures(unsigned cur_frame);
};

#endif

