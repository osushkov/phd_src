/*
 * ReconstructionFrame.h
 *
 *  Created on: 03/07/2009
 *      Author: osushkov
 */

#ifndef RECONSTRUCTIONFRAME_H_
#define RECONSTRUCTIONFRAME_H_

#include "../Features/SIFTFeature3D.h"
#include "../Util/Vector3D.h"
#include "../Util/Matrix.h"
#include "../Util/Transform.h"
#include "../Features/FeatureMemory/ObjectSnapshot.h"
#include <vector>
#include <list>
#include <map>

struct ReconstructionFeature {
    unsigned id;
    bool is_object;
    bool is_linked;

    SIFTFeature3D feature;
    Vector3D transformed_position;
    Vector3D final_position;
};

struct ReconstructionFrame {
    unsigned frame_id;

    std::vector<ReconstructionFeature> rfeatures;
    Transform transform;

    // (index,num_matches) pairs.
    std::vector< std::pair<unsigned,unsigned> > linked_frames;
    std::map<unsigned, std::vector<SnapshotMatchResult> > result_map;
    bool is_linked;

    ObjectSnapshot *snapshot;
};

struct BestFitFeaturePair {
    Vector3D fpos0; // scene
    Vector3D fpos1; // snapshot
    float pmatch;
};

#endif /* RECONSTRUCTIONFRAME_H_ */
