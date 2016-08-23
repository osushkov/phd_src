
#ifndef _SIFT_GENERATOR_H_
#define _SIFT_GENERATOR_H_

#include <vector>
#include <list>
#include <string>
#include <pthread.h>

#include "SIFT/sift.h"
#include "SIFT/imgfeatures.h"
#include "Util/Semaphore.h"


struct SIFTGeneratorStartData {
    std::vector<char> left_cam_buffer;
    std::vector<char> right_cam_buffer;
    unsigned frame_num;
};

struct SIFTGeneratorEndData {
    std::vector<feature> left_features;
    std::vector<feature> right_features;
    unsigned frame_num;
};


class SIFTGenerator {
  public:
    SIFTGenerator(std::string filename, bool read_from_file, unsigned cam_width, unsigned cam_height);
    ~SIFTGenerator();

    unsigned getNextFrame(std::vector<feature> &left_features, std::vector<feature> &right_features);
    void putNewFrame(const std::vector<char> &left_cam_buffer, const std::vector<char> &right_cam_buffer, 
                     unsigned frame_num);

  private:

    bool read_from_file, finished_file;
    unsigned cam_width, cam_height;

    std::list<SIFTGeneratorStartData> start_queue;
    std::list<SIFTGeneratorEndData> end_queue;

    Util::Semaphore lock;    
    Util::Semaphore start_semaphore;
    Util::Semaphore end_semaphore;

    std::string sift_filename;
    pthread_t working_thread;

    static void* producerThreadLive(void *thread_arg);
    static void* producerThreadFile(void *thread_arg);

    static bool readFrameFeatures(std::vector<feature> &left_sift_features, 
                                  std::vector<feature> &right_sift_features, 
                                  std::fstream &input_file);
    static bool writeFrameFeatures(const std::vector<feature> &left_sift_features, 
                                   const std::vector<feature> &right_sift_features,
                                   std::fstream &output_file, unsigned frame_num);
};

#endif

