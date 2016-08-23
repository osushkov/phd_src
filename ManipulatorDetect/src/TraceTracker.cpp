
#include "TraceTracker.h"
#include "StereoFeatureCorrelation.h"
#include "SIFT/imgfeatures.h"
#include "Util/Common.h"

#include <vector>
#include <cmath>
#include <cassert>

#define MAX_FRAME_GAP 3
#define MAX_FEATURE_MOVE_PER_FRAME 0.7f


TraceTracker::TraceTracker(){
    nmd_maxima = false;
    last_snapshot_frame = 0;
}


TraceTracker::~TraceTracker(){

}


void TraceTracker::submitStereoFeatures(std::vector<StereoFeature> features, unsigned cur_frame){
    latest_frame = cur_frame;
    frame_features[cur_frame] = features;
    
    for(unsigned i = 0; i < features.size(); i++){
        insertFeature(features[i], cur_frame);
    }

    // Purge all active traces which are too old.
    std::vector<unsigned> new_traces_ref;
    for(unsigned i = 0; i < cur_active_traces_ref.size(); i++){
        if(cur_frame - all_traces[cur_active_traces_ref[i]].nodes.back().frame < MAX_FRAME_GAP){
            new_traces_ref.push_back(cur_active_traces_ref[i]);
        }
    }
    cur_active_traces_ref = new_traces_ref;

    if(canFindLongTraces(last_snapshot_frame, cur_frame)){
        nmd_maxima = true;
        last_snapshot_frame = cur_frame-10;
    }
    
/*
    nmd_ratings.push_back(currentNMDRating());
    

    if(nmd_ratings.size() > 20){
        bool is_max = true;
        for(unsigned i = nmd_ratings.size()-20; i < nmd_ratings.size()-3; i++){
            if(nmd_ratings[i] > nmd_ratings[nmd_ratings.size()-3]){ is_max = false; }
        }
       
        for(unsigned i = nmd_ratings.size()-2; i < nmd_ratings.size(); i++){
            if(nmd_ratings[i] > nmd_ratings[nmd_ratings.size()-3]){ is_max = false; }
        }


        if(is_max){
            nmd_maxima = true;
        }
    }*/
}


std::vector<Trace> TraceTracker::getActiveTraces(void){
    std::vector<Trace> result;
    for(unsigned i = 0; i < cur_active_traces_ref.size(); i++){
        result.push_back(all_traces[cur_active_traces_ref[i]]);
    }
    return result;
}

bool TraceTracker::isNMDMaxima(unsigned &frame_start, std::vector<StereoFeature> &features_start,
                               unsigned &frame_end, std::vector<StereoFeature> &features_end){
    if(!nmd_maxima){
        return false;
    }

    std::vector<Trace> tmp, potential_traces = getPotentialTraces();
    frame_end = findBestEndFrame(potential_traces);
    
    for(unsigned i = 0; i < potential_traces.size(); i++){
        if(traceContainsFrame(potential_traces[i], frame_end)){
            tmp.push_back(potential_traces[i]);
        }
    }
    potential_traces = tmp;

    unsigned earliest_frame = latest_frame;
    for(unsigned i = 0; i < potential_traces.size(); i++){
        if(potential_traces[i].nodes.front().frame < earliest_frame){
            earliest_frame = potential_traces[i].nodes.front().frame;
        }
    }

    if(earliest_frame < frame_end - 20){ earliest_frame = frame_end - 20; }

    bool found = false;
    for(unsigned cur_frame = earliest_frame; cur_frame < frame_end; cur_frame++){
        unsigned num_traces_have_frame = 0;
        for(unsigned i = 0; i < potential_traces.size(); i++){
            if(traceContainsFrame(potential_traces[i], cur_frame)){
                num_traces_have_frame++;
            }
        }
        if(num_traces_have_frame > potential_traces.size()/3){
            frame_start = cur_frame;
            found = true;
            break;
        }
    }

    if(!found){ 
        nmd_maxima = false;
        return false;
    }

    tmp.clear();
    for(unsigned i = 0; i < potential_traces.size(); i++){
        if(traceContainsFrame(potential_traces[i], frame_start)){
            tmp.push_back(potential_traces[i]);
        }
    }
    potential_traces = tmp;

    features_start.clear();
    features_end.clear();

    for(unsigned i = 0; i < potential_traces.size(); i++){
        for(unsigned j = 0; j < potential_traces[i].nodes.size(); j++){
            if(potential_traces[i].nodes[j].frame == frame_start){
                features_start.push_back(potential_traces[i].nodes[j].feature);
            }
            else if(potential_traces[i].nodes[j].frame == frame_end){
                features_end.push_back(potential_traces[i].nodes[j].feature);
            }
        }
    }


    nmd_maxima = false;
    return true;
}

std::vector<StereoFeature> TraceTracker::getFrameFeatures(unsigned frame_num){
    if(frame_features.find(frame_num) == frame_features.end()){ 
        return std::vector<StereoFeature>(); 
    }
    else{
        return frame_features[frame_num];
    }
}

/* Insert Feature
 *
 * To insert a feature we have to find a trace of features which has a recent (within MAX_FRAME_GAP frames)
 * trace node at its head, and this head node must be "similar" to the given feature as well as within a 
 * certain distance radius in world space. 
 * 1. Go through each currently active trace.
 * 2. Check to see if the head feature of the trace is the closest feature of all co-occuring features for 
 *    that frame.
 * 3. If it is, check to see if it is within a maximum radius, if so, add the new feature to the front of the 
 *    trace.
 * 4. If no matching existing traces have been found, create a new trace from the given feature.
 * */

void TraceTracker::insertFeature(StereoFeature feature, unsigned cur_frame){
    std::map<unsigned, StereoFeature> closest_feature;

    TraceNode new_node;
    new_node.feature = feature;
    new_node.frame = cur_frame;

    bool found_chain = false;
    for(unsigned i = 0; i < cur_active_traces_ref.size(); i++){
        unsigned trace_frame_end = all_traces[cur_active_traces_ref[i]].nodes.back().frame;

        if(closest_feature.find(trace_frame_end) == closest_feature.end()){
            unsigned closest_index = 0;
            if(findClosestFeature(feature, frame_features[trace_frame_end], closest_index)){
                closest_feature[trace_frame_end] = frame_features[trace_frame_end][closest_index];
            }
        }
       
        float dist_radius = MAX_FEATURE_MOVE_PER_FRAME * (cur_frame - trace_frame_end);
        assert(dist_radius >= 0.0f);

        StereoFeature head_feature = all_traces[cur_active_traces_ref[i]].nodes.back().feature;

        if(dist(feature.position, closest_feature[trace_frame_end].position) < dist_radius &&
           areTheSame(head_feature, closest_feature[trace_frame_end])){
            // add to chain
            all_traces[cur_active_traces_ref[i]].nodes.push_back(new_node);
            found_chain = true;
            break;
        }
    }

    if(!found_chain){
        Trace new_trace;
        new_trace.nodes.push_back(new_node);
        all_traces.push_back(new_trace);
        cur_active_traces_ref.push_back(all_traces.size()-1);
    }
}

bool TraceTracker::findClosestFeature(StereoFeature feature, std::vector<StereoFeature> &all_features, 
                                      unsigned &closest_index){
    float closest_dist = FLT_MAX, scnd_closest_dist = FLT_MAX;
    
    for(unsigned i = 0; i < all_features.size(); i++){
        float dist = Common::featureDist(feature, all_features[i]);
        if(dist < closest_dist){
            scnd_closest_dist = closest_dist;
            closest_dist = dist;
            closest_index = i;
        }
        else if(dist < scnd_closest_dist){
            scnd_closest_dist = dist;
        }
    }

    return true; //closest_dist < 0.85f*scnd_closest_dist;
}


bool TraceTracker::areTheSame(StereoFeature &feature1, StereoFeature &feature2){
    return are_equal(feature1.feature_left, feature2.feature_left) &&
           are_equal(feature1.feature_right, feature2.feature_right);
}

float TraceTracker::dist(Vector3D v1, Vector3D v2){
    return sqrtf((v1.x-v2.x)*(v1.x-v2.x) + (v1.y-v2.y)*(v1.y-v2.y) +
                 (v1.z-v2.z)*(v1.z-v2.z));
}

float TraceTracker::currentNMDRating(void){
    float avrg_rating = 0.0f;
    for(unsigned i = 0; i < cur_active_traces_ref.size(); i++){
        avrg_rating += traceNMDRating(all_traces[cur_active_traces_ref[i]]);
    }
    return avrg_rating/(float)(cur_active_traces_ref.size());
}

float TraceTracker::traceNMDRating(Trace trace){
    float trace_length = trace.nodes.size();
    float dist_travelled = dist(trace.nodes[0].feature.position, trace.nodes.back().feature.position);
    return dist_travelled*trace_length;
}

std::vector<Trace> TraceTracker::getPotentialTraces(void){
    std::vector<Trace> result;

    // Get all currently active traces which are of at least a minimum size
    for(unsigned i = 0; i < cur_active_traces_ref.size(); i++){
        if(all_traces[cur_active_traces_ref[i]].nodes.back().frame - all_traces[cur_active_traces_ref[i]].nodes.front().frame > 10){
            result.push_back(all_traces[cur_active_traces_ref[i]]);
        }
    }

    // Cull out all traces which are shorter than the average length.
    float avrg_length = 0.0f;
    for(unsigned i = 0; i < result.size(); i++){
        avrg_length += dist(result[i].nodes.front().feature.position, result[i].nodes.back().feature.position);
    }
    avrg_length /= (float)(result.size());

    std::vector<Trace> tmp;
    for(unsigned i = 0; i < result.size(); i++){
        if(dist(result[i].nodes.front().feature.position, result[i].nodes.back().feature.position) > 1.5f*avrg_length){
            tmp.push_back(result[i]);
        }
    }
    result = tmp;

    return result;
}

unsigned TraceTracker::findBestEndFrame(const std::vector<Trace> &traces){
    std::map<unsigned, unsigned> counter;
    std::map<unsigned, unsigned>::iterator it;    

    for(unsigned i = 0; i < traces.size(); i++){
        for(unsigned j = 0; j < 4; j++){
            unsigned frame_num = traces[i].nodes[traces[i].nodes.size()-5-j].frame;
            if(counter.find(frame_num) == counter.end()){
                counter[frame_num] = 1;
            }
            else{
                counter[frame_num]++;
            }
        }
    }

    unsigned max_count = 0, max_frame= 0;
    for(it = counter.begin(); it != counter.end(); ++it){
        if(it->second > max_count){
            max_count = it->second;
            max_frame = it->first;
        }
    }

    return max_frame;
}

bool TraceTracker::traceContainsFrame(const Trace &trace, unsigned frame){
    for(unsigned i = 0; i < trace.nodes.size(); i++){
        if(trace.nodes[i].frame == frame){ return true; }
        else if(trace.nodes[i].frame > frame){ return false; }
    }
    return false;
}

bool TraceTracker::canFindLongTraces(unsigned frame_start, unsigned frame_end){
    if(frame_end - frame_start < 20){ return false; }

    unsigned num_long_enough = 0;
    for(unsigned i = 0; i < cur_active_traces_ref.size(); i++){
        Trace cur_trace = all_traces[cur_active_traces_ref[i]];
        for(unsigned j = 0; j < cur_trace.nodes.size(); j++){
            if(cur_trace.nodes[j].frame > frame_start){
                if(dist(cur_trace.nodes[j].feature.position, cur_trace.nodes.back().feature.position) > 3.0f){
                    num_long_enough++;
                    break;
                }
            }
        }
    }

    
    return num_long_enough > 4;
}

