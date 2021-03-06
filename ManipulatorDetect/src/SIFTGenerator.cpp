
#include "SIFTGenerator.h"

#include <cassert>
#include <pthread.h>
#include <fstream>
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "Util/Common.h"
#include "Util/ParallelServer.h"


struct SIFTWorkerData {
    SIFTWorkerData(IplImage *_img, std::vector<feature> &_dump_buffer):
        img(_img), dump_buffer(_dump_buffer) {}

    IplImage *img;
    std::vector<feature> &dump_buffer;
};

class SIFTWorker : public ParallelExecutor {
  public:

    SIFTWorker(){};
    ~SIFTWorker(){};

    void performTask(void *task_data, unsigned rank, unsigned size){
        SIFTWorkerData *data = (SIFTWorkerData *)task_data;
        Common::extractFeatures(data->img, data->dump_buffer);
    }
};

////////////////////////////////////////////////////////////////////

SIFTGenerator::SIFTGenerator(std::string filename, bool read_from_file, 
                             unsigned cam_width, unsigned cam_height) :
    read_from_file(read_from_file), finished_file(false),
    cam_width(cam_width), cam_height(cam_height), lock(1) {

    sift_filename = filename;

    if(read_from_file){
        pthread_create(&working_thread, NULL, &SIFTGenerator::producerThreadFile, (void *)this);
    }
    else{
        pthread_create(&working_thread, NULL, &SIFTGenerator::producerThreadLive, (void *)this);
    }
}

SIFTGenerator::~SIFTGenerator(){

}

unsigned SIFTGenerator::getNextFrame(std::vector<feature> &left_features, 
                                     std::vector<feature> &right_features){
    if(read_from_file && finished_file && end_queue.size() == 0){ 
        return 0;
    }

    end_semaphore.wait();
    
    lock.wait();
    left_features = end_queue.front().left_features;
    right_features = end_queue.front().right_features;
    unsigned frame_num = end_queue.front().frame_num;

    end_queue.pop_front();
    lock.signal();
    
    return frame_num;
}

void SIFTGenerator::putNewFrame(const std::vector<char> &left_cam_buffer, const std::vector<char> &right_cam_buffer, 
                                unsigned frame_num){
    SIFTGeneratorStartData new_start_data;
    new_start_data.left_cam_buffer = left_cam_buffer;
    new_start_data.right_cam_buffer = right_cam_buffer;
    new_start_data.frame_num = frame_num;

    lock.wait();
    start_queue.push_back(new_start_data);
    lock.signal();    

    start_semaphore.signal();
}

void* SIFTGenerator::producerThreadLive(void *thread_arg){
    assert(thread_arg != NULL);
    SIFTGenerator *parent = (SIFTGenerator *)thread_arg;

    std::fstream sift_file(parent->sift_filename.c_str(),  std::ios::binary|std::ios::out);

    IplImage* img_left = cvCreateImage(cvSize(parent->cam_width, parent->cam_height),IPL_DEPTH_8U,3);
    IplImage* img_right = cvCreateImage(cvSize(parent->cam_width, parent->cam_height),IPL_DEPTH_8U,3);

    ParallelServer pserver(PSM_PIPELINE);
    SIFTWorker worker_thread;

    while(1){
        parent->start_semaphore.wait(); // wait for a new frame to come in so we can process it

        SIFTGeneratorEndData new_end_data;
        SIFTGeneratorStartData cur_data = parent->start_queue.front();

        parent->lock.wait();
        parent->start_queue.pop_front();
        parent->lock.signal();

        new_end_data.frame_num = cur_data.frame_num;

        Util::Semaphore task_sem;

        img_left->imageData = (char*)&(cur_data.left_cam_buffer[0]);
        img_right->imageData = (char*)&(cur_data.right_cam_buffer[0]);

        SIFTWorkerData data1(img_left, new_end_data.left_features), data2(img_right, new_end_data.right_features);
        ParallelTask task1(&worker_thread, &data1, &task_sem), task2(&worker_thread, &data2, &task_sem);

        pserver.executeParallelTask(task1);
        pserver.executeParallelTask(task2);

        task_sem.wait();
        task_sem.wait();

        writeFrameFeatures(new_end_data.left_features, new_end_data.right_features, sift_file, new_end_data.frame_num);

        parent->lock.wait();
        parent->end_queue.push_back(new_end_data);
        parent->lock.signal();

        parent->end_semaphore.signal();
    }

    pthread_exit((void *)NULL);
}

void* SIFTGenerator::producerThreadFile(void *thread_arg){
    assert(thread_arg != NULL);
    SIFTGenerator *parent = (SIFTGenerator *)thread_arg;

    std::fstream sift_file(parent->sift_filename.c_str(),  std::ios::binary|std::ios::in);

    SIFTGeneratorEndData new_end_data;
    new_end_data.frame_num = 0;
    while(readFrameFeatures(new_end_data.left_features, new_end_data.right_features, sift_file)){
        parent->lock.wait();
        parent->end_queue.push_back(new_end_data);
        parent->lock.signal();

        parent->end_semaphore.signal();
        new_end_data.frame_num++;

        while(parent->end_queue.size() > 500){
            while(parent->end_queue.size() > 200){
                sleep(1);
            }
        }

        //std::cout << parent->end_queue.size() << std::endl;
    }

    parent->finished_file = true;
    pthread_exit((void *)NULL);    
}

bool SIFTGenerator::readFrameFeatures(std::vector<feature> &left_sift_features, 
                                      std::vector<feature> &right_sift_features, 
                                      std::fstream &input_file){
    unsigned frame_num;
    unsigned num_left_features, num_right_features;

    if(input_file.eof() || !input_file.good()){ return false; }

    left_sift_features.clear();
    right_sift_features.clear();

    input_file.read((char*)&frame_num, sizeof(unsigned));
    input_file.read((char*)&num_left_features, sizeof(unsigned));
    input_file.read((char*)&num_right_features, sizeof(unsigned));
   
    if(input_file.eof() || !input_file.good()){ return false; } 

    for(unsigned i = 0; i < num_left_features; i++){
        feature cur_feature;
        read_feature(input_file, cur_feature);
        left_sift_features.push_back(cur_feature);
    }

    for(unsigned i = 0; i < num_right_features; i++){
        feature cur_feature;
        read_feature(input_file, cur_feature);
        right_sift_features.push_back(cur_feature);
    }
    
    if(input_file.eof() || !input_file.good()){ return false; }
    return true;
}

bool SIFTGenerator::writeFrameFeatures(const std::vector<feature> &left_sift_features, 
                                       const std::vector<feature> &right_sift_features,
                                       std::fstream &output_file, unsigned frame_num){
    output_file.write((char*)&frame_num, sizeof(unsigned));
    unsigned lsize = left_sift_features.size(), rsize = right_sift_features.size();
    output_file.write((char*)&lsize, sizeof(unsigned));
    output_file.write((char*)&rsize, sizeof(unsigned));

    for(unsigned i = 0; i < left_sift_features.size(); i++){
        write_feature(output_file, left_sift_features[i]);
    }
    for(unsigned i = 0; i < right_sift_features.size(); i++){
        write_feature(output_file, right_sift_features[i]);
    }

    return true;
}

