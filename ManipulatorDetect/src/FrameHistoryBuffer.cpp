
#include "FrameHistoryBuffer.h"


FrameHistoryBuffer::FrameHistoryBuffer(unsigned max_size){
    msize = max_size;
}

FrameHistoryBuffer::~FrameHistoryBuffer(){

}

void FrameHistoryBuffer::addToBuffer(const std::vector<char> &left_frame, const std::vector<char> &right_frame, 
                                     unsigned frame_num){

    FrameHistoryElem new_elem;
    new_elem.left_frame = left_frame;
    new_elem.right_frame = right_frame;
    new_elem.frame_num = frame_num;
    
    history.push_front(new_elem);

    while(history.size() > msize){
        history.pop_back();
    }
}

bool FrameHistoryBuffer::getHistoryFrame(std::vector<char> &left_frame, std::vector<char> &right_frame, 
                                         unsigned frame_num){

    std::list<FrameHistoryElem>::iterator it;
    for(it = history.begin(); it != history.end(); ++it){
        if(it->frame_num == frame_num){
            left_frame = it->left_frame;
            right_frame = it->right_frame;
            return true;
        }
    }
    
    return false;
}

