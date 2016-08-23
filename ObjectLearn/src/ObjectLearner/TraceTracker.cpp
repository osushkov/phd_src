
#include "TraceTracker.h"
#include "../Features/StereoFeatureCorrelation.h"
#include "../Features/SIFT/imgfeatures.h"
#include "../Util/Common.h"
#include "../Util/Timer.h"
#include "../Util/PerfStats.h"
#include "../Util/Geometry.h"

#include <vector>
#include <cmath>
#include <cassert>

#define MAX_FRAME_GAP_VALID_TRACE 3
#define MAX_FEATURE_MOVE_PER_FRAME 1.5f
#define MIN_TRACE_LENGTH 4.0f


static std::vector<float> bins[60];
float total_length = 0.0f;
float total_num = 0;

std::vector<float> object_trajectory_diffs;


TraceTracker::TraceTracker(){
    last_snapshot_frame = 0;
}


TraceTracker::~TraceTracker(){

}


void TraceTracker::submitStereoFeatures(std::vector<StereoFeature> features,
                                        std::vector<bool> is_arm_flags,
                                        unsigned cur_frame){
    assert(features.size() == is_arm_flags.size());

    latest_frame = cur_frame;
    frame_features[cur_frame] = features;

    for(unsigned i = 0; i < features.size(); i++){
        insertFeature(features[i], is_arm_flags[i], cur_frame);
    }

    // Purge all active traces which are too old.
    std::vector<unsigned> new_traces_ref;
    for(unsigned i = 0; i < cur_active_traces_ref.size(); i++){
        if(!isTraceTooOld(all_traces[cur_active_traces_ref[i]], cur_frame)){
            new_traces_ref.push_back(cur_active_traces_ref[i]);
        }
    }

    cur_active_traces_ref = new_traces_ref;

    /*
    for(unsigned i = 0; i < 60; i++){
        std::cout << Common::average(bins[i]) << " " << Common::standardDeviation(bins[i]) << std::endl;
    }
    std::cout << std::endl;
    */

}


std::vector<Trace> TraceTracker::getActiveTraces(void){
    std::vector<Trace> result;
    for(unsigned i = 0; i < cur_active_traces_ref.size(); i++){
        result.push_back(all_traces[cur_active_traces_ref[i]]);
        /*if(isArmTrace(all_traces[cur_active_traces_ref[i]])){
            result.back().tag = 1;
        }
        else{
            result.back().tag = 0;
        }*/
    }
    return result;
}


bool TraceTracker::shouldCorrelate(unsigned &frame_start,
                                   std::vector<StereoFeature> &features_start,
                                   unsigned &frame_end,
                                   std::vector<StereoFeature> &features_end,
                                   unsigned cur_frame){

    Util::Timer timer("TraceTracker::shouldCorrelate");
    timer.start();

    if(cur_frame - last_snapshot_frame < 7){
        timer.stop();
        return false;
    }

    //purgeArmTraces();

    unsigned num_moved_traces = 0;
    std::vector<Trace> long_traces;
    for(unsigned i = 0; i < cur_active_traces_ref.size(); i++){
        if(traceLength(all_traces[cur_active_traces_ref[i]]) > MIN_TRACE_LENGTH &&
           (all_traces[cur_active_traces_ref[i]].nodes.front().feature.position - all_traces[cur_active_traces_ref[i]].nodes.back().feature.position).length() > MIN_TRACE_LENGTH/2.0f &&
           all_traces[cur_active_traces_ref[i]].nodes.size() > 5 &&
           cur_frame - all_traces[cur_active_traces_ref[i]].nodes.back().frame <= 1 &&
           !isArmTrace(all_traces[cur_active_traces_ref[i]])){

            num_moved_traces +=
            (int)(traceLengthBetweenFrames(all_traces[cur_active_traces_ref[i]], cur_frame, cur_frame-15) > 0.5f);

            long_traces.push_back(all_traces[cur_active_traces_ref[i]]);
        }
    }

    //long_traces = purgeInvalidTraces(long_traces);

    if(long_traces.size() < 4 || num_moved_traces < 4){
        return false;
    }

    frame_start = cur_frame - 10;
    frame_end = cur_frame;

    features_start.clear();
    features_end.clear();
    for(unsigned i = 0; i < long_traces.size(); i++){
        features_start.push_back(long_traces[i].nodes.front().feature);
        features_end.push_back(long_traces[i].nodes.back().feature);
    }
    last_snapshot_frame = cur_frame;


    /*
    for(unsigned i = 0; i < cur_active_traces_ref.size(); i++){
        if(isArmTrace(all_traces[cur_active_traces_ref[i]])){ continue; }
        if((all_traces[cur_active_traces_ref[i]].nodes.front().feature.position - all_traces[cur_active_traces_ref[i]].nodes.back().feature.position).length() < MIN_TRACE_LENGTH){
            continue;
        }

        for(unsigned j = i; j < cur_active_traces_ref.size(); j++){
            if(isArmTrace(all_traces[cur_active_traces_ref[j]])){ continue; }
            if((all_traces[cur_active_traces_ref[j]].nodes.front().feature.position - all_traces[cur_active_traces_ref[j]].nodes.back().feature.position).length() < MIN_TRACE_LENGTH){
                continue;
            }

            object_trajectory_diffs.push_back(traceDistance(all_traces[cur_active_traces_ref[i]], all_traces[cur_active_traces_ref[j]]));
        }
    }

    std::cout << "TD: " << Common::average(object_trajectory_diffs) << " " << Common::standardDeviation(object_trajectory_diffs) << std::endl;
*/
    return true;
}


std::vector<StereoFeature> TraceTracker::getFrameFeatures(unsigned frame_num){
    if(frame_features.find(frame_num) == frame_features.end()){
        return std::vector<StereoFeature>();
    }
    else{
        assert(frame_features.find(frame_num) != frame_features.end());
        return frame_features[frame_num];
    }
}


bool TraceTracker::findClosestFeature(StereoFeature feature,
                                      std::vector<StereoFeature> &all_features,
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


void TraceTracker::printBins(void){
    std::cout << "***" << std::endl;
    std::cout << total_length << " " << total_num << std::endl;
    std::cout << "***" << std::endl;
}


/* Insert Feature
 *
 * To insert a feature we have to find a trace of features which has a recent
 * (within MAX_FRAME_GAP_VALID_TRACE frames) trace node at its head, and this head
 * node must be "similar" to the given feature as well as within a certain distance
 * radius in world space.
 * 1. Go through each currently active trace.
 * 2. Check to see if the head feature of the trace is the closest feature of all
 *    co-occuring features for that frame.
 * 3. If it is, check to see if it is within a maximum radius, if so, add the new
 *    feature to the front of the trace.
 * 4. If no matching existing traces have been found, create a new trace from the
 *    given feature.
 */
void TraceTracker::insertFeature(StereoFeature feature, bool is_arm, unsigned cur_frame){
    Util::Timer timer("TraceTracker::insertFeature");
    timer.start();

    std::map<unsigned, StereoFeature> closest_feature;

    TraceNode new_node;
    new_node.feature = feature;
    new_node.frame = cur_frame;
    new_node.is_arm = is_arm;


    float min_stereo_dist = 0.0f;
    unsigned best_trace_index = 0;
    bool found_trace = false;

    for(unsigned i = 0; i < cur_active_traces_ref.size(); i++){
        StereoFeature lead_feature = all_traces[cur_active_traces_ref[i]].nodes.back().feature;
        float dist = (lead_feature.position-feature.position).length();
        unsigned frame_diff = cur_frame - all_traces[cur_active_traces_ref[i]].nodes.back().frame;

        float lori_dist =  Geometry::minAngleDistance(lead_feature.feature_left.ori, feature.feature_left.ori);
        float rori_dist = Geometry::minAngleDistance(lead_feature.feature_right.ori, feature.feature_right.ori);

        if(dist < MAX_FEATURE_MOVE_PER_FRAME*frame_diff &&
           lori_dist < frame_diff*11.0f*M_PI/180.0f &&
           rori_dist < frame_diff*11.0f*M_PI/180.0f){

            //std::cout << "~ " << lori_dist << " " << rori_dist << std::endl;
            float stereo_dist = Common::featureDist(feature, lead_feature);
            if(!found_trace || stereo_dist < min_stereo_dist){
                found_trace = true;
                min_stereo_dist = stereo_dist;
                best_trace_index = i;
            }

        }
    }

    if(found_trace && min_stereo_dist < 200.0f){
/*
        std::vector<TraceNode> nodes = all_traces[cur_active_traces_ref[best_trace_index]].nodes;
        if((feature.position - nodes.front().feature.position).length() > 2.0f &&
            !isArmTrace(all_traces[cur_active_traces_ref[best_trace_index]])){

        for(unsigned i = 0; i < nodes.size(); i++){
            float dist = Geometry::minAngleDistance(feature.feature_left.ori, nodes[i].feature.feature_left.ori) +
                         Geometry::minAngleDistance(feature.feature_right.ori, nodes[i].feature.feature_right.ori);
            dist = 0.5f * 180.0f/M_PI * dist;
            //float dist = Common::featureDist(feature, nodes[i].feature);
            unsigned fdist = cur_frame - nodes[i].frame;

            if(fdist < 60){
                bins[fdist].push_back(dist);
            }
        }
        }
*/

        all_traces[cur_active_traces_ref[best_trace_index]].nodes.push_back(new_node);
    }
    else{
        Trace new_trace;
        new_trace.tag = 0;
        new_trace.nodes.push_back(new_node);
        all_traces.push_back(new_trace);
        cur_active_traces_ref.push_back(all_traces.size()-1);
        updateLengthBins(new_trace, feature);
    }

    timer.stop();
}

bool TraceTracker::areTheSame(StereoFeature feature1, StereoFeature feature2){
    return are_equal(feature1.feature_left, feature2.feature_left) &&
           are_equal(feature1.feature_right, feature2.feature_right);
}

float TraceTracker::traceLength(const Trace &trace){
    float length = 0.0f;
    for(unsigned i = 0; i < trace.nodes.size()-1; i++){
        Vector3D diff_vec = trace.nodes.at(i).feature.position -
                            trace.nodes.at(i+1).feature.position;

        length += diff_vec.length();
    }
    return length;
}

float TraceTracker::traceLengthBetweenFrames(const Trace &trace,
                                             unsigned start_frame,
                                             unsigned end_frame){
    for(unsigned i = 0; i < trace.nodes.size(); i++){
        if(trace.nodes[i].frame <= end_frame){
            return (trace.nodes.front().feature.position - trace.nodes[i].feature.position).length();
        }
    }

    return (trace.nodes.front().feature.position - trace.nodes.back().feature.position).length();
}

void TraceTracker::updateLengthBins(const Trace &trace, const StereoFeature &feature){
    unsigned bin_index = 4.0f*(feature.feature_left.scl + feature.feature_right.scl)/2.0f;
    if(bin_index >= 128){ return; }

    //total_length += traceLength(trace);
    //total_num++;
}

bool TraceTracker::isTraceTooOld(const Trace &trace, unsigned cur_frame){
    return (cur_frame - trace.nodes.back().frame) > MAX_FRAME_GAP_VALID_TRACE;
}

void TraceTracker::gatherTraceStatistics(void){
    float sum = 0.0f;
    float max = 0.0f;
    unsigned num_pairs = 0;

    for(unsigned i = 0; i < cur_active_traces_ref.size()-1; i++){
        for(unsigned j = i+1; j < cur_active_traces_ref.size(); j++){
            num_pairs++;
            float dist = traceDistance(all_traces[cur_active_traces_ref[i]],
                                       all_traces[cur_active_traces_ref[j]]);

            sum += dist;
            if(dist > max){
                max = dist;
            }

        }
    }

    std::cout << max << " " << (sum/num_pairs) << std::endl;
}

float TraceTracker::traceDistance(const Trace &trace1, const Trace &trace2){
    // Not sure if i need this.
    Trace ctrace1 = trace1; //generateContiguousTrace(trace1);
    Trace ctrace2 = trace2; //generateContiguousTrace(trace2);

    std::vector<Vector3D> path1, path2;

    unsigned j = 0;
    for(unsigned i = 0; i < ctrace1.nodes.size(); i++){
        while(j < ctrace2.nodes.size() && ctrace2.nodes.at(j).frame < ctrace1.nodes.at(i).frame){
            j++;
        }

        if(j < ctrace2.nodes.size() && ctrace2.nodes.at(j).frame == ctrace1.nodes.at(i).frame){
            path1.push_back(ctrace1.nodes.at(i).feature.position);
            path2.push_back(ctrace2.nodes.at(j).feature.position);
        }
    }

    return Common::curveDistance(path1, path2);
}

bool TraceTracker::isArmTrace(const Trace &trace){
    if(trace.nodes.size() < 3){
        return false;
    }

    unsigned num_arm_features = 0;
    for(unsigned i = 0; i < trace.nodes.size(); i++){
        if(trace.nodes[i].is_arm){
            num_arm_features++;
        }
    }

    return num_arm_features > trace.nodes.size()/3;
}

std::vector<Trace> TraceTracker::purgeInvalidTraces(const std::vector<Trace> &traces){
    std::vector<Trace> result, arm_traces;

    /*if(cur_active_arm_traces_ref.size() < 3){
        return result;
    }
    */

    for(unsigned j = 0; j < cur_active_traces_ref.size(); j++){
        if(isArmTrace(all_traces[cur_active_traces_ref[j]]) && all_traces[cur_active_traces_ref[j]].nodes.size() > 5){
            arm_traces.push_back(all_traces[cur_active_traces_ref[j]]);
        }
    }

    if(arm_traces.size() < 1){
        return result;
    }

    for(unsigned i = 0; i < traces.size(); i++){
        unsigned num_close_to = 0;
        for(unsigned j = 0; j < arm_traces.size(); j++){
           if(traceDistance(traces[i], arm_traces[j]) < 0.5f){
               num_close_to++;
           }

           if(num_close_to >= 2){
               result.push_back(traces[i]);
               break;
           }
        }
    }

    return result;
}

void TraceTracker::purgeArmTraces(void){
    std::vector<unsigned> new_ref;
    cur_active_arm_traces_ref.clear();

    for(unsigned i = 0; i < cur_active_traces_ref.size(); i++){
        if(isArmTrace(all_traces[cur_active_traces_ref[i]])){
            cur_active_arm_traces_ref.push_back(i);
        }
        else{
            new_ref.push_back(i);
        }
    }

    cur_active_traces_ref = new_ref;
}
