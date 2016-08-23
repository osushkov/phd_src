
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

void FrameHistoryBuffer::addToBuffer(const std::vector<unsigned char> &left_frame,
                                     const std::vector<unsigned char> &right_frame,
                                     const std::vector<StereoFeature> &features,
                                     const std::vector<float> &arm_joints,
                                     unsigned frame_num){

    FrameHistoryElem *new_elem = new FrameHistoryElem;
    new_elem->left_frame = left_frame;
    new_elem->right_frame = right_frame;
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

bool FrameHistoryBuffer::getHistoryFrame(std::vector<unsigned char> &left_frame,
                                         std::vector<unsigned char> &right_frame,
                                         std::vector<StereoFeature> &features,
                                         unsigned frame_num){

    std::map<unsigned, FrameHistoryElem*>::iterator it;
    it = history_frames.find(frame_num);
    if(it == history_frames.end()){
        return false;
    }
    else {
        left_frame = it->second->left_frame;
        right_frame = it->second->right_frame;
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
