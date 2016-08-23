
#include "TraceTracker.h"
#include "../Arm/ArmForwardKinematics.h"
#include "../Features/SIFTFeature3D.h"
#include "../Features/SIFT/imgfeatures.h"
#include "../Util/Common.h"
#include "../Util/Timer.h"
#include "../Util/PerfStats.h"
#include "../Util/Geometry.h"

#include <vector>

#define _USE_MATH_DEFINES
#include <math.h>
#include <cassert>

#define MAX_FRAME_GAP_VALID_TRACE 3
//1.5
#define MAX_FEATURE_MOVE_PER_FRAME 4.0f
#define MAX_FEATURE_ROTATE_PER_FRAME (10.0f*M_PI/180.0f)
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


void TraceTracker::submitStereoFeatures(std::vector<SIFTFeature3D> features,
                                        std::vector<float> arm_joints,
                                        unsigned cur_frame){

    latest_frame = cur_frame;
    frame_features[cur_frame] = features;

    addDummyArmTraces(arm_joints, cur_frame);

    for(unsigned i = 0; i < features.size(); i++){
        insertFeature(features[i], cur_frame);
    }

    // Purge all active traces which are too old.
    std::vector<unsigned> new_traces_ref;
    for(unsigned i = 0; i < cur_active_traces_ref.size(); i++){
        if(!isTraceTooOld(all_traces[cur_active_traces_ref[i]], cur_frame)){
            new_traces_ref.push_back(cur_active_traces_ref[i]);
        }
    }

    cur_active_traces_ref = new_traces_ref;
}


std::vector<Trace> TraceTracker::getActiveTraces(void){
    std::vector<Trace> result;
    for(unsigned i = 0; i < cur_active_traces_ref.size(); i++){
        result.push_back(all_traces[cur_active_traces_ref[i]]);
    }
    return result;
}


bool TraceTracker::shouldCorrelate(unsigned &frame_start,
                                   std::vector<SIFTFeature3D> &features_start,
                                   unsigned &frame_end,
                                   std::vector<SIFTFeature3D> &features_end,
                                   std::vector<Trace> &paths,
                                   unsigned cur_frame){

    Util::Timer timer;
    timer.start();

    if(cur_frame - last_snapshot_frame < 7){
        timer.stop();
        return false;
    }

    const unsigned frame_dist = 15;

    unsigned num_moved_traces = 0;
    std::vector<Trace> long_traces;
    for(unsigned i = 0; i < cur_active_traces_ref.size(); i++){
        if(traceLength(all_traces[cur_active_traces_ref[i]]) > MIN_TRACE_LENGTH &&
           (all_traces[cur_active_traces_ref[i]].nodes.front().feature.position - all_traces[cur_active_traces_ref[i]].nodes.back().feature.position).length() > MIN_TRACE_LENGTH &&
           all_traces[cur_active_traces_ref[i]].nodes.size() > 5 &&
           cur_frame - all_traces[cur_active_traces_ref[i]].nodes.back().frame <= 1 &&
           !isTraceCrooked(all_traces[cur_active_traces_ref[i]], cur_frame-frame_dist)){

            num_moved_traces +=
            (int)(traceLengthBetweenFrames(all_traces[cur_active_traces_ref[i]], cur_frame, cur_frame-frame_dist) > 0.5f);

            long_traces.push_back(all_traces[cur_active_traces_ref[i]]);
        }
    }

    //std::cout << long_traces.size() << " : ";
    long_traces = purgeInvalidTraces(long_traces, cur_frame-frame_dist);
    //std::cout << long_traces.size() << std::endl;

    if(long_traces.size() < 4 || num_moved_traces < 4){
        return false;
    }

    frame_start = cur_frame - frame_dist;
    frame_end = cur_frame;

    features_start.clear();
    features_end.clear();
    paths.clear();
    for(unsigned i = 0; i < long_traces.size(); i++){
        features_start.push_back(long_traces[i].nodes.front().feature);
        features_end.push_back(long_traces[i].nodes.back().feature);
        paths.push_back(long_traces[i]);
    }
    last_snapshot_frame = cur_frame;

    return true;
}


std::vector<SIFTFeature3D> TraceTracker::getFrameFeatures(unsigned frame_num){
    if(frame_features.find(frame_num) == frame_features.end()){
        return std::vector<SIFTFeature3D>();
    }
    else{
        assert(frame_features.find(frame_num) != frame_features.end());
        return frame_features[frame_num];
    }
}


bool TraceTracker::findClosestFeature(SIFTFeature3D feature,
                                      std::vector<SIFTFeature3D> &all_features,
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
void TraceTracker::insertFeature(SIFTFeature3D feature, unsigned cur_frame){
    std::map<unsigned, SIFTFeature3D> closest_feature;

    TraceNode new_node;
    new_node.feature = feature;
    new_node.frame = cur_frame;

    float min_stereo_dist = 1000.0f;
    unsigned best_trace_index = 0;
    bool found_trace = false;

    for(unsigned i = 0; i < cur_active_traces_ref.size(); i++){
        SIFTFeature3D lead_feature = all_traces[cur_active_traces_ref[i]].nodes.back().feature;
        float dist = (lead_feature.position-feature.position).length();
        unsigned frame_diff = cur_frame - all_traces[cur_active_traces_ref[i]].nodes.back().frame;

        float ori_dist =  Geometry::minAngleDistance(lead_feature.sift_feature.ori, feature.sift_feature.ori);

        if(dist < MAX_FEATURE_MOVE_PER_FRAME*frame_diff &&
           ori_dist < frame_diff*MAX_FEATURE_ROTATE_PER_FRAME){

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
}

bool TraceTracker::areTheSame(SIFTFeature3D feature1, SIFTFeature3D feature2){
    return are_equal(feature1.sift_feature, feature2.sift_feature);
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

void TraceTracker::updateLengthBins(const Trace &trace, const SIFTFeature3D &feature){
    unsigned bin_index = (unsigned)(4.0f*(feature.sift_feature.scl + feature.sift_feature.scl)/2.0f);
    if(bin_index >= 128){ return; }

    //total_length += traceLength(trace);
    //total_num++;
}

bool TraceTracker::isTraceTooOld(const Trace &trace, unsigned cur_frame){
    return (cur_frame - trace.nodes.back().frame) > MAX_FRAME_GAP_VALID_TRACE;
}

bool TraceTracker::isTraceCrooked(const Trace &trace, unsigned frame_start){
    const float check_dist = 2.5f;
    for(unsigned i = 0; i < trace.nodes.size(); i++){
        if(trace.nodes[i].frame < frame_start){ continue; }

        unsigned next_node = i;
        for(unsigned j = next_node+1; j < trace.nodes.size(); j++){
            if((trace.nodes[i].feature.position - trace.nodes[j].feature.position).length() >= check_dist){
                next_node = j;
                break;
            }
        }
        if(!next_node == i){ continue; }

        unsigned next_next_node = next_node;
        for(unsigned j = next_next_node+1; j < trace.nodes.size(); j++){
            if((trace.nodes[next_node].feature.position - trace.nodes[j].feature.position).length() >= check_dist){
                next_next_node = j;
                break;
            }
        }
        if(!next_next_node == next_node){ continue; }

        Vector3D v0 = trace.nodes[next_node].feature.position - trace.nodes[i].feature.position;
        Vector3D v1 = trace.nodes[next_next_node].feature.position - trace.nodes[next_node].feature.position;
        v0.normalise();
        v1.normalise();

        if(fabs(acosf(v0.dotProduct(v1))) > 130.0f*M_PI/180.0f){
            return true;
        }
    }

    return false;
}

void TraceTracker::gatherTraceStatistics(void){
    float sum = 0.0f;
    float max = 0.0f;
    unsigned num_pairs = 0;

    for(unsigned i = 0; i < cur_active_traces_ref.size()-1; i++){
        for(unsigned j = i+1; j < cur_active_traces_ref.size(); j++){
            num_pairs++;
            float dist = traceDistance(all_traces[cur_active_traces_ref[i]],
                                       all_traces[cur_active_traces_ref[j]], 0);

            sum += dist;
            if(dist > max){
                max = dist;
            }

        }
    }

    std::cout << max << " " << (sum/num_pairs) << std::endl;
}

float TraceTracker::traceDistance(const Trace &trace1, const Trace &trace2, unsigned first_frame){
    // Not sure if i need this.
    Trace ctrace1 = trace1; //generateContiguousTrace(trace1);
    Trace ctrace2 = trace2; //generateContiguousTrace(trace2);

    float min_dist = 1000.0f;

    for(int offset = -10; offset <= 10; offset++){
        std::vector<Vector3D> path1, path2;
        unsigned j = 0;

        Trace atrace1;
        for(unsigned i = 0; i < ctrace1.nodes.size(); i++){
            TraceNode node = ctrace1.nodes[i];
            if((int)node.frame >= -offset){
                node.frame += offset;
                atrace1.nodes.push_back(node);
            }
        }

        for(unsigned i = 0; i < atrace1.nodes.size(); i++){
            if(atrace1.nodes[i].frame < first_frame){
                continue;
            }

            while(j < ctrace2.nodes.size() && ctrace2.nodes.at(j).frame < atrace1.nodes.at(i).frame){
                j++;
            }

            if(j < ctrace2.nodes.size() && ctrace2.nodes.at(j).frame == atrace1.nodes.at(i).frame){
                path1.push_back(atrace1.nodes.at(i).feature.position);
                path2.push_back(ctrace2.nodes.at(j).feature.position);
            }
        }

        if(path1.size() > 3){
            float dist = Common::curveDistance2(path1, path2);
            if(dist < min_dist){
                min_dist = dist;
            }
        }
    }

    
    //std::cout << "dist: " << min_dist << std::endl;
    //getchar();

    return min_dist;
}

std::vector<Trace> TraceTracker::purgeInvalidTraces(const std::vector<Trace> &traces, unsigned first_frame){
    std::vector<Trace> result;

    for(unsigned i = 0; i < traces.size(); i++){
        unsigned num_close_to = 0;
        for(unsigned j = 0; j < dummy_arm_traces.size(); j++){
            float dist = traceDistance(traces[i], dummy_arm_traces[j], traces[i].nodes.front().frame);
            if(dist < 0.5f){
                num_close_to++;
            }  
        }

        if(num_close_to > 0){
            result.push_back(traces[i]);
        }
    }

    return result;
}

void TraceTracker::addDummyArmTraces(std::vector<float> arm_joints, unsigned frame_num){
    ArmForwardKinematics afk;
    std::vector<Vector3D> gripper_pose = afk.getArmPoint(arm_joints);

    const float offset_length = 5.0f;
    std::vector<SIFTFeature3D> dummy_features;

    SIFTFeature3D dummy_feature0, dummy_feature1, dummy_feature2, dummy_feature3;
    dummy_feature0.position = Common::armPointToWorldSpace(gripper_pose[0] + offset_length*gripper_pose[1]);
    dummy_feature1.position = Common::armPointToWorldSpace(gripper_pose[0] + offset_length*gripper_pose[2]);
    dummy_feature2.position = Common::armPointToWorldSpace(gripper_pose[0] + offset_length*gripper_pose[3]);
    dummy_feature3.position = Common::armPointToWorldSpace(gripper_pose[0]);

    dummy_features.push_back(dummy_feature0);
    dummy_features.push_back(dummy_feature1);
    dummy_features.push_back(dummy_feature2);
    dummy_features.push_back(dummy_feature3);

    std::vector<TraceNode> nodes;
    for(unsigned i = 0; i < dummy_features.size(); i++){
        TraceNode node;
        node.feature = dummy_features[i];
        node.frame = frame_num;

        nodes.push_back(node);
    }

    if(dummy_arm_traces.size() == 0){
        for(unsigned i = 0; i < nodes.size(); i++){
            Trace trace;
            trace.nodes.push_back(nodes[i]);
            dummy_arm_traces.push_back(trace);
        }
    }
    else{
        for(unsigned i = 0; i < nodes.size(); i++){
            dummy_arm_traces[i].nodes.push_back(nodes[i]);
        }
    }
}