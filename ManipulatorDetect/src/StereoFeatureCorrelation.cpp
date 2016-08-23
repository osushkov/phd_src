
#include "StereoFeatureCorrelation.h"
#include "Util/Common.h"
#include "Util/ParallelServer.h"
#include "Util/Semaphore.h"
#include "SIFT/imgfeatures.h"

#include <vector>


#define EPIPOLAR_OFFSET_LIMIT (2.0f)
#define LEFT_CAMERA_X_OFFSET (-6.0f)
#define RIGHT_CAMERA_X_OFFSET (6.0f)
#define FEATURE_MAX_ORI_DIST (15.0f)


struct CorrelationWorkerData {
    CorrelationWorkerData(const std::vector<feature> &left,
                          std::vector<feature> &right,
                          std::vector<StereoFeature> &result):
        result_lock(1), left(left), right(right), result(result) {}


    Util::Semaphore result_lock;
    
    const std::vector<feature> &left;
    std::vector<feature> &right;
    std::vector<StereoFeature> &result;
};

static bool getClosestFeature(feature f, const std::vector<feature> &all_features, unsigned &index);
static StereoFeature getStereoFeature(feature left_feature, feature right_feature);

class CorrelationWorker : public ParallelExecutor {
  public:

    CorrelationWorker(){};
    ~CorrelationWorker(){};

    void performTask(void *task_data, unsigned rank, unsigned size){
        CorrelationWorkerData *data = (CorrelationWorkerData*)task_data;

        std::vector<StereoFeature> partial_result;
        unsigned begin = (data->left.size()*rank)/size;
        unsigned end   = (data->left.size()*(rank+1))/size;
       
        for(unsigned i = begin; i < end; i++){
        
            unsigned r_index = 0, l_index = 0;
            if(getClosestFeature(data->left[i], data->right, r_index) && 
               getClosestFeature(data->right[r_index], data->left, l_index) &&
               l_index == i){
           
                // Check to see that the orientations of the two distance matched 
                // features are close, if not then move on
                float ori1 = data->left[l_index].ori * 180.0f/M_PI;
                float ori2 = data->right[r_index].ori * 180.0f/M_PI;

                // normalise the orientations to 0..360 range
                while(ori1 < 0.0f){ ori1 += 360.0f; }
                while(ori2 < 0.0f){ ori2 += 360.0f; }
                while(ori1 >= 360.0f){ ori1 -= 360.0f; }
                while(ori2 >= 360.0f){ ori2 -= 360.0f; }

                float diff1 = fabs(ori1-ori2);
                float diff2 = fabs((ori1+360.0f) - ori2);
                float diff3 = fabs(ori1 - (ori2+360.0f));

                if(diff1 < FEATURE_MAX_ORI_DIST || diff2 < FEATURE_MAX_ORI_DIST || diff3 < FEATURE_MAX_ORI_DIST){           
                    partial_result.push_back(getStereoFeature(data->left[l_index], data->right[r_index]));
                }

                //right[r_index] = right.back();
                //right.pop_back();            
            }
        }

        data->result_lock.wait();
        for(unsigned i = 0; i < partial_result.size(); i++){
            data->result.push_back(partial_result[i]);
        }
        data->result_lock.signal();

    }

};

ParallelServer pserver(PSM_PARALLEL);
CorrelationWorker worker_thread;

bool getClosestFeature(feature f, const std::vector<feature> &all_features, unsigned &index){
    feature cur_closest, cur_2nd_closest;
    float closest_dist_sq = FLT_MAX, sclosest_dist_sq = FLT_MAX;

    for(unsigned i = 0; i < all_features.size(); i++){

        // Skip features which are not epipolar.
        if(fabs(f.y - all_features[i].y) > EPIPOLAR_OFFSET_LIMIT){ continue; }

        float dist_sq = 0.0f;

        for(unsigned j = 0; j < 128; j++){
            float diff = f.descr[j] - all_features[i].descr[j];
            dist_sq += diff*diff;
            if(dist_sq > sclosest_dist_sq){ break; }
        }
    
        if(dist_sq < closest_dist_sq){
            closest_dist_sq = dist_sq;
            cur_closest = all_features[i];
            index = i;
        }
        else if(dist_sq < sclosest_dist_sq){
            sclosest_dist_sq = dist_sq;
            cur_2nd_closest = all_features[i];
        }
    }

    bool result = false;
    if(sqrtf(closest_dist_sq) < sqrtf(sclosest_dist_sq)*0.8f){ result = true; }

    return result;
}


StereoFeature getStereoFeature(feature left_feature, feature right_feature){
    Vector3D far_left = Common::getProjectedCameraPoint(left_feature.x, left_feature.y);
    Vector3D far_right = Common::getProjectedCameraPoint(right_feature.x, right_feature.y);
    
    Vector3D origin_left, origin_right;
    origin_left.y = origin_left.z = origin_right.y = origin_right.z = 0.0f;
    origin_left.x = LEFT_CAMERA_X_OFFSET;
    origin_right.x = RIGHT_CAMERA_X_OFFSET;

    far_left.x += LEFT_CAMERA_X_OFFSET;
    far_right.x += RIGHT_CAMERA_X_OFFSET;

    Vector3D r1, r2;
    r1.x = r2.x = r1.y = r2.y = r1.z = r2.z = 0.0f;
    float d1 = 0.0f, d2 = 0.0f;
    Common::lineIntersect(origin_left, far_left, origin_right, far_right, r1, r2, d1, d2);

    Vector3D avrg;
    avrg.x = (r1.x + r2.x)/2.0f;
    avrg.y = (r1.y + r2.y)/2.0f;
    avrg.z = (r1.z + r2.z)/2.0f;

    StereoFeature result;
    result.feature_left = left_feature;
    result.feature_right = right_feature;
    result.position = avrg;

    return result;
}

std::vector<StereoFeature> StereoFeatureCorrelation::correlateFeatures(std::vector<feature> left, 
                                                                       std::vector<feature> right){
    std::vector<StereoFeature> result;

    Util::Semaphore task_sem;
    CorrelationWorkerData data(left, right, result);
    ParallelTask task(&worker_thread, &data, &task_sem);

    pserver.executeParallelTask(task);

    task_sem.wait();
    /*

    for(unsigned i = 0; i < left.size(); i++){
        
        unsigned r_index = 0, l_index = 0;
        if(getClosestFeature(left[i], right, r_index) && 
           getClosestFeature(right[r_index], left, l_index) &&
           l_index == i){
           
            // Check to see that the orientations of the two distance matched 
            // features are close, if not then move on
            float ori1 = left[l_index].ori * 180.0f/M_PI;
            float ori2 = right[r_index].ori * 180.0f/M_PI;

            // normalise the orientations to 0..360 range
            while(ori1 < 0.0f){ ori1 += 360.0f; }
            while(ori2 < 0.0f){ ori2 += 360.0f; }
            while(ori1 >= 360.0f){ ori1 -= 360.0f; }
            while(ori2 >= 360.0f){ ori2 -= 360.0f; }
 
            float diff1 = fabs(ori1-ori2);
            float diff2 = fabs((ori1+360.0f) - ori2);
            float diff3 = fabs(ori1 - (ori2+360.0f));

            if(diff1 < FEATURE_MAX_ORI_DIST || diff2 < FEATURE_MAX_ORI_DIST || diff3 < FEATURE_MAX_ORI_DIST){           
                result.push_back(getStereoFeature(left[l_index], right[r_index]));
            }

            right[r_index] = right.back();
            right.pop_back();            
        }
    }*/

    return result;
}

