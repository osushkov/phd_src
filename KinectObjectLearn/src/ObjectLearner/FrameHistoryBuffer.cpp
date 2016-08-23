
#include "FrameHistoryBuffer.h"


FrameHistoryBuffer::FrameHistoryBuffer(unsigned max_size){
    msize = max_size;
}

FrameHistoryBuffer::~FrameHistoryBuffer() {
    std::map<unsigned, FrameHistoryElem*>::iterator it;
    for(it = history_frames.begin(); it!= history_frames.end(); ++it){
        delete it->second;
    }
}

void FrameHistoryBuffer::addToBuffer(const KinectCamera::CorrelatedImage &correlated_frame,
                                     const std::vector<SIFTFeature3D> &features,
                                     const std::vector<float> &arm_joints,
                                     unsigned frame_num){

    FrameHistoryElem *new_elem = new FrameHistoryElem();
    new_elem->correlated_frame = correlated_frame;
    new_elem->features = features;
    new_elem->arm_joints = arm_joints;
    new_elem->frame_num = frame_num;

    history_frames[frame_num] = new_elem;
    history_queue.push_front(frame_num);

    while(history_queue.size() > msize){
        delete history_frames[history_queue.back()];
        history_frames.erase(history_queue.back());
        history_queue.pop_back();
    }
}

bool FrameHistoryBuffer::getHistoryFrame(KinectCamera::CorrelatedImage &correlated_frame,
                                         std::vector<SIFTFeature3D> &features,
                                         unsigned frame_num){

    std::map<unsigned, FrameHistoryElem*>::iterator it;
    it = history_frames.find(frame_num);
    if(it == history_frames.end()){
        return false;
    }
    else {
        correlated_frame = it->second->correlated_frame;
        features = it->second->features;
        return true;
    }
}

bool FrameHistoryBuffer::getHistoryJoints(std::vector<float> &arm_joints, unsigned frame_num){
    std::map<unsigned, FrameHistoryElem*>::iterator it;
    it = history_frames.find(frame_num);
    if(it == history_frames.end()){
        return false;
    }
    else {
        arm_joints = it->second->arm_joints;
        return true;
    }
}
