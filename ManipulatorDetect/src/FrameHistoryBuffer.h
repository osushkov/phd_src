
#ifndef _FrameHistoryBuffer_H_
#define _FrameHistoryBuffer_H_

#include <vector>
#include <list>


struct FrameHistoryElem {
    std::vector<char> left_frame, right_frame;
    unsigned frame_num;
};

class FrameHistoryBuffer {
  public:
    FrameHistoryBuffer(unsigned max_size);
    ~FrameHistoryBuffer();

    void addToBuffer(const std::vector<char> &left_frame, const std::vector<char> &right_frame, 
                     unsigned frame_num);

    bool getHistoryFrame(std::vector<char> &left_frame, std::vector<char> &right_frame, unsigned frame_num);

  private:
        
    unsigned msize;
    std::list<FrameHistoryElem> history;

};

#endif

