
#include <iostream>
#include <cstdio>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include <fstream>

#include "RemoteCamera.h"
#include "PlaybackCamera.h"
#include "SIFT/sift.h"
#include "SIFT/imgfeatures.h"
#include "Util/Common.h"
#include "Util/ParallelServer.h"
#include "Keypoint.h"
#include "Timer.h"


std::vector< std::vector<Keypoint> > frame_keypoints;
unsigned cur_frame = 0;


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


bool getClosestKeypoint(Keypoint kp, const std::vector<Keypoint> &all_keypoints, Keypoint &closest_keypoint){
    Keypoint cur_closest, cur_2nd_closest;
    double closest_dist_sq = DBL_MAX, sclosest_dist_sq = DBL_MAX;

    for(unsigned i = 0; i < all_keypoints.size(); i++){
        double dist_sq = 0.0f;

        for(unsigned j = 0; j < 128; j++){
            double diff = kp.feature_left.descr[j] - all_keypoints[i].feature_left.descr[j];
            dist_sq += diff*diff;
            if(dist_sq > sclosest_dist_sq){ break; }
        }
    
        if(dist_sq < closest_dist_sq){
            closest_dist_sq = dist_sq;
            cur_closest = all_keypoints[i];
        }
        else if(dist_sq < sclosest_dist_sq){
            sclosest_dist_sq = dist_sq;
            cur_2nd_closest = all_keypoints[i];
        }
    }

    closest_keypoint = cur_closest;

    bool result = false;
    if(sqrt(closest_dist_sq) < sqrt(sclosest_dist_sq)*0.8){ result = true; }

    return result;
}

float dist(Vector3D var1, Vector3D var2){
    return sqrtf((var1.x-var2.x)*(var1.x-var2.x) + (var1.y-var2.y)*(var1.y-var2.y) + (var1.z-var2.z)*(var1.z-var2.z));
}


void displayTrace(IplImage* left_frame, IplImage* right_frame){
    if(cur_frame < 6){ return; }

    Keypoint closest, target;
    //float sum = 0.0f;;
    //unsigned num = 0;
    //unsigned look_back[5] = {0};
    //cvPoint prev = cvPoint(0.0f, 0.0f);
    unsigned num_missed = 0;

    for(unsigned i = 0; i < frame_keypoints.back().size(); i++){
        target = frame_keypoints[frame_keypoints.size()-1].at(i);
        num_missed = 0;
        for(unsigned j = 1; j < frame_keypoints.size(); j++){
            if(j > 20){ break; }

            if(getClosestKeypoint(target, frame_keypoints[frame_keypoints.size()-1-j], closest) && 
               dist(target.position, closest.position) < 1.5f && 
               dist(target.position, closest.position) > 0.1f &&
               num_missed <= 1){

                cvLine(left_frame, cvPoint(target.feature_left.x, target.feature_left.y), 
                   cvPoint(closest.feature_left.x, closest.feature_left.y),
	               CV_RGB(255, 0, 0), 1, 8, 0);

                target = closest;
                num_missed = 0;
            }
            else{
                num_missed++;
            }
        }
    }

/*
    for(unsigned i = 0; i < frame_keypoints.back().size(); i++){
        target = frame_keypoints[frame_keypoints.size()-1].at(i);
        if(getClosestKeypoint(target, frame_keypoints[frame_keypoints.size()-2], closest) && 
           dist(target.position, closest.position) < 1.5f && 
           dist(target.position, closest.position) > 0.3f){
            cvLine(left_frame, cvPoint(target.feature_left.x, target.feature_left.y), 
                   cvPoint(closest.feature_left.x, closest.feature_left.y),
	               CV_RGB(255, 0, 0), 1, 8, 0);

            sum += dist(target.position, closest.position);
            //std::cout << dist(target.position, closest.position) << std::endl;
            num++;
        }
        else if(getClosestKeypoint(target, frame_keypoints[frame_keypoints.size()-3], closest) && 
                dist(target.position, closest.position) < 3.0f &&
                dist(target.position, closest.position) > 0.6f){
            cvLine(left_frame, cvPoint(target.feature_left.x, target.feature_left.y), 
                   cvPoint((closest.feature_left.x+target.feature_left.x)/2.0f, (closest.feature_left.y+target.feature_left.y)/2.0f),
	               CV_RGB(255, 0, 0), 1, 8, 0);

            sum += dist(target.position, closest.position)/2.0f;
            //std::cout << dist(target.position, closest.position)/2.0f << std::endl;
            num++;
        }
        
        for(unsigned j = 0; j < 5; j++){
            if(getClosestKeypoint(target, frame_keypoints[frame_keypoints.size()-2-j], closest) &&
               dist(target.position, closest.position) < 4.5f){
                look_back[j]++;
            }
        }
    }

    if(num > 0){
        //std::cout << sum/num << std::endl;
    }

    std::cout << frame_keypoints.back().size() << "\t";
    for(unsigned i = 0; i < 5; i++){
        std::cout << look_back[i] << "\t";
    }
    std::cout << std::endl;*/
}


void cullNonMatches(std::vector<feature> &left_features, std::vector<feature> &right_features){
    feature tmp1, tmp2;
    std::vector<feature> result_left, result_right;
   
    Vector3D origin_left, origin_right;
    origin_left.x = origin_left.y = origin_left.z = 0.0f;
    origin_right = origin_left;

    origin_left.x = -6.0f;
    origin_right.x = 6.0f;
 
    for(unsigned i = 0; i < left_features.size(); i++){
        if(Common::getClosestFeature(left_features[i], right_features, tmp1) && 
           Common::getClosestFeature(tmp1, left_features, tmp2) &&
           left_features[i].x == tmp2.x && left_features[i].y == tmp2.y &&
           fabs(left_features[i].y-tmp1.y) <= 2.0f){
           
            float ori1 = left_features[i].ori * 180.0f/M_PI;
            float ori2 = tmp1.ori * 180.0f/M_PI;

            while(ori1 < 0.0f){ ori1 += 360.0f; }
            while(ori2 < 0.0f){ ori2 += 360.0f; }
 
            float diff1 = fabs(ori1-ori2);
            float diff2 = fabs((ori1+360.0f) - ori2);
            float diff3 = fabs(ori1 - (ori2+360.0f));
            if(diff1 > 15.0f && diff2 > 15.0f && diff3 > 15.0f){ continue; }            




            result_left.push_back(left_features[i]);
            result_right.push_back(tmp1);

            Vector3D far_left = Common::getProjectedCameraPoint(left_features[i].x, left_features[i].y);
            Vector3D far_right = Common::getProjectedCameraPoint(tmp1.x, tmp1.y);

            far_left.x += origin_left.x;
            far_left.y += origin_left.y;
            far_left.z += origin_left.z;

            far_right.x += origin_right.x;
            far_right.y += origin_right.y;
            far_right.z += origin_right.z;


            Vector3D r1, r2;
            r1.x = r2.x = r1.y = r2.y = r1.z = r2.z = 0.0f;
            float d1 = 0.0f, d2 = 0.0f;
            Common::lineIntersect(origin_left, far_left, origin_right, far_right, r1, r2, d1, d2);

            Vector3D avrg;
            avrg.x = (r1.x + r2.x)/2.0f;
            avrg.y = (r1.y + r2.y)/2.0f;
            avrg.z = (r1.z + r2.z)/2.0f;
          
 
            Keypoint new_keypoint;
            new_keypoint.position = avrg;
            new_keypoint.feature_left = left_features[i];
            new_keypoint.feature_right = tmp1;
            frame_keypoints[cur_frame].push_back(new_keypoint);
 
        }
    }

    left_features = result_left;
    right_features = result_right;
}


void writeFeatures(const std::vector<feature> &left_sift_features, const std::vector<feature> &right_sift_features,
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
}

void readFeatures(std::vector<feature> &left_sift_features, std::vector<feature> &right_sift_features, 
                  std::fstream &input_file){
    unsigned frame_num;
    unsigned num_left_features, num_right_features;

    left_sift_features.clear();
    right_sift_features.clear();

    input_file.read((char*)&frame_num, sizeof(unsigned));
    input_file.read((char*)&num_left_features, sizeof(unsigned));
    input_file.read((char*)&num_right_features, sizeof(unsigned));
    
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
}



int main(int argc, char **argv){
    Camera *cam = new PlaybackCamera("data/vid1/");
   
    unsigned cam_width = cam->getCameraImageWidth();
    unsigned cam_height = cam->getCameraImageHeight();

    IplImage* img_left = cvCreateImage(cvSize(cam_width, cam_height),IPL_DEPTH_8U,3);
    IplImage* img_right = cvCreateImage(cvSize(cam_width, cam_height),IPL_DEPTH_8U,3);

    cvNamedWindow("Left Eye", 1);
    cvNamedWindow("Right Eye", 1);

    std::vector<char> left_cam_buffer, right_cam_buffer;
    left_cam_buffer.resize(cam_width*cam_height*3);
    right_cam_buffer.resize(cam_width*cam_height*3);

    std::vector<feature> left_sift_features, right_sift_features;

    ParallelServer pserver(PSM_PIPELINE);
    SIFTWorker worker_thread;

    std::fstream sift_file("data/sift_file.dat", std::ios::in | std::ios::binary);

    unsigned last_frame = 0;
    readFeatures(left_sift_features, right_sift_features, sift_file);    

    while(cam->isActive()){
        frame_keypoints.push_back(std::vector<Keypoint>());        

        char key = cvWaitKey(2);		//Need this to update the image windows.
        if(key == 'q'){
            break;
        }

        // Get the image buffers from the remote camera
        cam->getLeftImage(left_cam_buffer);
        cam->getRightImage(right_cam_buffer);

        img_left->imageData = (char*)&(left_cam_buffer[0]);
        img_right->imageData = (char*)&(right_cam_buffer[0]);

/*
        Util::Timer timer;
        timer.start();
        Util::Semaphore task_sem;
        SIFTWorkerData data1(img_left, left_sift_features), data2(img_right, right_sift_features);

        ParallelTask task1(&worker_thread, &data1, &task_sem), task2(&worker_thread, &data2, &task_sem);

        pserver.executeParallelTask(task1);
        pserver.executeParallelTask(task2);

        task_sem.wait();
        task_sem.wait();*/

        while(cam->getFrameNum() != last_frame){
            readFeatures(left_sift_features, right_sift_features, sift_file);
            last_frame++;
        }
/*
        timer.stop();
        std::cout << timer.getNumElapsedSeconds()*1000.0f << std::endl;

        writeFeatures(left_sift_features, right_sift_features, sift_file, cur_frame);
*/
        cullNonMatches(left_sift_features, right_sift_features);       
 
        draw_features(img_left, (feature *)&(left_sift_features[0]), left_sift_features.size());
        draw_features(img_right, (feature *)&(right_sift_features[0]), right_sift_features.size());

        displayTrace(img_left, img_right);

        cvShowImage("Left Eye", img_left);
        cvShowImage("Right Eye", img_right);

        cur_frame++;
        ((PlaybackCamera*)cam)->nextFrame();
    }

    return 0;
}

