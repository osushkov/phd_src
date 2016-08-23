
#include "SIFTGenerator.h"

#include <cassert>
#include <pthread.h>
#include <fstream>
#include <iostream>
#include <list>

#ifdef _WIN32
#include <cv.h>
#include <highgui.h>
#else
#include <opencv/cv.h>
#include <opencv/highgui.h>
#endif

#include "../Util/Common.h"
#include "../Util/ParallelServer.h"
#include "../Util/Timer.h"

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

SIFTGenerator::SIFTGenerator(SIFTGeneratorMode mode, bool write_to_file, std::string filename,
                             unsigned cam_width, unsigned cam_height) :
    mode(mode), write_to_file(write_to_file), finished_file(false),
    cam_width(cam_width), cam_height(cam_height), lock(1),
    end_lock(0), should_end(false) {

    sift_filename = filename;

    if(mode == LOCAL_FILE_READ){
        pthread_create(&working_thread, NULL, &SIFTGenerator::producerThreadFile, (void *)this);
    } else if(mode == LOCAL_COMPUTE){
        pthread_create(&working_thread, NULL, &SIFTGenerator::producerThreadLiveLocal, (void *)this);
    }
}

SIFTGenerator::~SIFTGenerator(){
    should_end = true;
    start_semaphore.signal();
    end_lock.wait();

    //TODO: why the hell couldnt i use and iterator here?!
    while(start_queue.size() != 0){
        delete start_queue.front();
        start_queue.pop_front();
    }

    while (end_queue.size() != 0) {
        delete end_queue.front();
        end_queue.pop_front();
    }
}

unsigned SIFTGenerator::getNextFrame(std::vector<feature> &left_features,
                                     std::vector<feature> &right_features){
    if(mode == LOCAL_FILE_READ && finished_file && end_queue.size() == 0){
        return 0;
    }

    end_semaphore.wait();

    lock.wait();
    left_features = end_queue.front()->left_features;
    right_features = end_queue.front()->right_features;
    unsigned frame_num = end_queue.front()->frame_num;

    delete end_queue.front();
    end_queue.pop_front();
    lock.signal();

    return frame_num;
}

void SIFTGenerator::putNewFrame(const std::vector<unsigned char> &left_cam_buffer,
                                const std::vector<unsigned char> &right_cam_buffer,
                                unsigned frame_num){
    if(mode == LOCAL_FILE_READ){ return; }

    SIFTGeneratorStartData *new_start_data = new SIFTGeneratorStartData;
    new_start_data->left_cam_buffer = left_cam_buffer;
    new_start_data->right_cam_buffer = right_cam_buffer;
    new_start_data->frame_num = frame_num;

    lock.wait();
    start_queue.push_back(new_start_data);
    lock.signal();

    start_semaphore.signal();
}

bool SIFTGenerator::haveOutstandingWork(void){
    bool result;

    lock.wait();
    result = start_queue.size() > 0;
    lock.signal();

    return result;
}

void* SIFTGenerator::producerThreadLiveLocal(void *thread_arg){
    assert(thread_arg != NULL);
    SIFTGenerator *parent = (SIFTGenerator *)thread_arg;

    std::fstream sift_file;
    if(parent->write_to_file){
        sift_file.open(parent->sift_filename.c_str(),  std::ios::binary|std::ios::out);
    }

    ParallelServer pserver(PSM_PIPELINE);
    SIFTWorker worker_thread;

    while(!parent->should_end){
        parent->start_semaphore.wait(); // wait for a new frame to come in so we can process it
        if(parent->should_end){
            break;
        }
        SIFTGeneratorEndData *new_end_data = new SIFTGeneratorEndData;
        SIFTGeneratorStartData cur_data = *(parent->start_queue.front());

        parent->lock.wait();
        delete parent->start_queue.front();
        parent->start_queue.pop_front();
        parent->lock.signal();

        new_end_data->frame_num = cur_data.frame_num;

        Util::Semaphore task_sem;

        IplImage* img_left = Common::imageFromBuffer(cur_data.left_cam_buffer,
                                                     parent->cam_width,
                                                     parent->cam_height);
        IplImage* img_right = Common::imageFromBuffer(cur_data.right_cam_buffer,
                                                      parent->cam_width,
                                                      parent->cam_height);

        SIFTWorkerData data1(img_left, new_end_data->left_features);
        SIFTWorkerData data2(img_right, new_end_data->right_features);

        ParallelTask task1(&worker_thread, &data1, &task_sem);
        ParallelTask task2(&worker_thread, &data2, &task_sem);

        Util::Timer timer; timer.start();
        pserver.executeParallelTask(task1);
        pserver.executeParallelTask(task2);

        task_sem.wait();
        task_sem.wait();
        timer.stop();

        if(parent->write_to_file){
            writeFrameFeatures(new_end_data->left_features, new_end_data->right_features,
                               sift_file, new_end_data->frame_num);
        }

        parent->lock.wait();
        parent->end_queue.push_back(new_end_data);
        parent->lock.signal();

        parent->end_semaphore.signal();

        cvReleaseImage(&img_left);
        cvReleaseImage(&img_right);
    }

    parent->end_lock.signal();
    pthread_exit((void *)NULL);

    return NULL;
}

unsigned allocateSIFTServer(unsigned job, unsigned total_jobs, std::vector<int> server_ranges){
    float frac = (float)job/(float)total_jobs;
    int p = (int)(frac*100);

    for(unsigned i = 0; i < server_ranges.size(); i++){
        if(p <= server_ranges[i]){ return i; }
    }
    return 0;
}

void* SIFTGenerator::producerThreadFile(void *thread_arg){
    assert(thread_arg != NULL);
    SIFTGenerator *parent = (SIFTGenerator *)thread_arg;

    std::fstream sift_file(parent->sift_filename.c_str(),  std::ios::binary|std::ios::in);
    unsigned frame_num = 0;

    while(!parent->should_end){
        SIFTGeneratorEndData *new_end_data = new SIFTGeneratorEndData;
        new_end_data->frame_num = frame_num;

        if(!readFrameFeatures(new_end_data->left_features, new_end_data->right_features, sift_file)){
            delete new_end_data;
            break;
        }

        parent->lock.wait();
        parent->end_queue.push_back(new_end_data);
        parent->lock.signal();

        parent->end_semaphore.signal();
        frame_num++;

        while(parent->end_queue.size() > 200){
            while(parent->end_queue.size() > 100){
#ifdef _WIN32
                Sleep(1000);
#else
                sleep(1);
#endif
                if(parent->should_end){
                    goto end;
                }
            }
        }

    }
end:

    parent->finished_file = true;
    parent->end_lock.signal();
    pthread_exit((void *)NULL);

    return NULL;
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

