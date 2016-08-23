
#include "StereoFeatureCorrelation.h"
#include "../Util/Common.h"
#include "../Util/Geometry.h"
#include "../Util/ParallelServer.h"
#include "../Util/Semaphore.h"
#include "../Util/Timer.h"
#include "SIFT/imgfeatures.h"

#include <vector>
#include <limits.h>
#include <list>

#define EPIPOLAR_OFFSET_LIMIT (1.5f)
#define FEATURE_MAX_ORI_DIST (7.0f)
#define MAX_FEATURE_DIST 300.0f

unsigned total = 0;
unsigned scale_buckets[128] = {0};


struct SIFTHintRow {
    unsigned row_index; // the y coordinate.
    std::list<std::pair<float,float> > brackets; // left_x,right_x pairs, sorted by left_x
};


struct CorrelationWorkerData {
    CorrelationWorkerData(const std::vector< std::vector<feature> > &left,
                          const std::vector< std::vector<feature> > &right,
                          std::vector<StereoFeature> &result):
        result_lock(1), left(left), right(right), result(result) {}

    Util::Semaphore result_lock;

    const std::vector< std::vector<feature> > &left;
    const std::vector< std::vector<feature> > &right;
    std::vector<StereoFeature> &result;
};

static bool getClosestFeature(feature f, const std::vector<feature> &all_features, unsigned &index);
static StereoFeature getStereoFeature(feature left_feature, feature right_feature);
static void buildFeaturePool(const std::vector< std::vector<feature> > &all_features, int row,
                             std::vector<feature> &result_pool);

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

            std::vector<feature> left_features, right_features;
            buildFeaturePool(data->left, i, left_features);
            buildFeaturePool(data->right, i, right_features);
            //std::cout << "!!" << left_features.size() << " " << right_features.size() << std::endl;

            for(unsigned j = 0; j < data->left.at(i).size(); j++){

                unsigned r_index = 0, l_index = 0;
                if(getClosestFeature(data->left.at(i).at(j), right_features, r_index) &&
                   getClosestFeature(right_features[r_index], left_features, l_index)){

                    if(left_features[l_index].x < right_features[r_index].x){ continue; }
                    float ori_diff = Geometry::minAngleDistance(left_features[l_index].ori,
                                                                right_features[r_index].ori);

                    if(ori_diff*180.0f/M_PI < FEATURE_MAX_ORI_DIST){
                        partial_result.push_back(getStereoFeature(left_features[l_index], right_features[r_index]));
                    }

                }
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
    //std::cout << all_features.size() << std::endl;
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

    bool result = true;
    //if(sqrtf(closest_dist_sq) < sqrtf(sclosest_dist_sq)*0.99f){ result = true; }
    if(closest_dist_sq > MAX_FEATURE_DIST*MAX_FEATURE_DIST){ result = false; }

    return result;
}


StereoFeature getStereoFeature(feature left_feature, feature right_feature){
    Vector3D position = Common::stereoProject(Vector2D(left_feature.x, left_feature.y),
                                              Vector2D(right_feature.x, right_feature.y));

    StereoFeature result;
    result.feature_left = left_feature;
    result.feature_right = right_feature;
    result.position = position;

    return result;
}

void buildFeaturePool(const std::vector< std::vector<feature> > &all_features, int row,
                      std::vector<feature> &result_pool){

    unsigned int start_row = 0;
    unsigned int end_row = row + EPIPOLAR_OFFSET_LIMIT;

    if(row >= EPIPOLAR_OFFSET_LIMIT){
        start_row = row - EPIPOLAR_OFFSET_LIMIT;
    }

    if(end_row >= all_features.size()){
        end_row = all_features.size()-1;
    }

    result_pool.clear();

    //std::cout << start_row << " " << row << " " << end_row << std::endl;
    for(unsigned row = start_row; row <= end_row; row++){
        for(unsigned i = 0; i < all_features.at(row).size(); i++){
            result_pool.push_back(all_features.at(row).at(i));
        }
    }
}

std::vector<StereoFeature> StereoFeatureCorrelation::correlateFeatures(std::vector<feature> left,
                                                                       std::vector<feature> right,
                                                                       unsigned res_height){

    total = 0;
    std::vector< std::vector<feature> > left_feature_rows(res_height);
    std::vector< std::vector<feature> > right_feature_rows(res_height);

    for(unsigned i = 0; i < left.size(); i++){
        left_feature_rows.at((int)left[i].y).push_back(left[i]);
    }

    for(unsigned i = 0; i < right.size(); i++){
        right_feature_rows.at((int)right[i].y).push_back(right[i]);
    }

    std::vector<StereoFeature> result;

    Util::Semaphore task_sem;
    CorrelationWorkerData data(left_feature_rows, right_feature_rows, result);
    ParallelTask task(&worker_thread, &data, &task_sem);

    pserver.executeParallelTask(task);

    task_sem.wait();
/*
    for(unsigned i = 0; i < 128; i++){
        std::cout << i/2.0f << "," << 100.0f * (float)scale_buckets[i]/(float)total << std::endl;
    }
    std::cout << std::endl;

    std::cout << "TOTAL: " << total << std::endl;*/
    return result;
}

std::vector<Vector3D> getPixelRegion(IplImage *img, unsigned cx, unsigned cy){
    std::vector<Vector3D> region;
    for(unsigned y = cy-3; y <= cy+3; y++){
        for(unsigned x = cx-5; x <= cx+5; x++){
            double r, g, b;
            Common::getPixel(img, x, y, r, g, b);
            region.push_back(Vector3D(r/255.0f, g/255.0f, b/255.0f));
        }
    }
    return region;
}

float pixelRegionDist(const std::vector<Vector3D> &region1, const std::vector<Vector3D> &region2,
                      const float hint){
    assert(region1.size() == region2.size());

    float result = 0.0f;
    for(unsigned i = 0; i < region1.size(); i++){
        result += (region1[i] - region2[i]).length()/region1.size();
        if(result > hint){
            return result;
        }
    }

    return result;
}

bool findFeature(IplImage *img_left, IplImage *img_right, unsigned lx, unsigned ly,
                 int disparity_estimate, SceneFeature &new_feature){
    float best_dist = FLT_MAX;
    float best_x = lx;

    std::vector<Vector3D> target_region = getPixelRegion(img_left, lx, ly);

    int startx = lx+disparity_estimate-75;
    if(startx < 1){ startx = 1; }

    int endx = lx+disparity_estimate+75;
    if(endx > img_right->width-1){
        endx = img_right->width-1;
    }

    //startx = 1;
    //endx = img_right->width-1;

    for(int ox = startx; ox < endx; ox++){
        std::vector<Vector3D> cur_region = getPixelRegion(img_right, ox, ly);
        float dist = pixelRegionDist(target_region, cur_region, 600);

        if(dist < best_dist){
            best_dist = dist;
            best_x = ox;
        }
    }

    if(best_dist < 600){
        new_feature.left_pos = Vector2D(lx, ly);
        new_feature.right_pos = Vector2D(best_x, ly);
        new_feature.position = Common::stereoProject(new_feature.left_pos, new_feature.right_pos);
        return true;
    }
    else{
        return false;
    }
}

unsigned disparityEstimate(const std::vector<StereoFeature> &features, unsigned x, unsigned y){
    Vector2D ppos(x, y);
    float closest_dist = FLT_MAX;
    unsigned disparity_estimate = 0;

    for(unsigned i = 0; i < features.size(); i++){
        Vector2D rpos(features[i].feature_left.x, features[i].feature_left.y);
        float dist = (ppos-rpos).length2();
        if(dist < closest_dist){
            closest_dist = dist;
            disparity_estimate = features[i].feature_right.x - features[i].feature_left.x;
        }
    }

    return disparity_estimate;
}

float calculateRegionDiff(IplImage *img_left, IplImage *img_right, Vector2D lp, Vector2D rp, float hint=FLT_MAX){
    std::vector<Vector3D> left_region = getPixelRegion(img_left, lp.x, lp.y);
    std::vector<Vector3D> right_region = getPixelRegion(img_right, rp.x, rp.y);

    return pixelRegionDist(left_region, right_region, hint);
}

void insertHintFeature(SceneFeature scene_feature, SIFTHintRow &hint_row){
    std::list<std::pair<float,float> >::iterator it;
    std::pair<float,float> new_entry(scene_feature.left_pos.x, scene_feature.right_pos.x);
    for(it = hint_row.brackets.begin(); it != hint_row.brackets.end(); ++it){
        if(it->first > new_entry.first){
            hint_row.brackets.insert(it, new_entry);
            return;
        }

    }
    hint_row.brackets.push_back(new_entry);
}

std::vector<SIFTHintRow> buildHintRows(const std::vector<SceneFeature> &scene_features){
    std::vector<SIFTHintRow> result;
    for(unsigned i = 0; i < 384; i++){ // TODO: query the resolution from settings.
        SIFTHintRow new_row;
        new_row.row_index = i;
        result.push_back(new_row);
    }

    for(unsigned i = 0; i < scene_features.size(); i++){
        unsigned feature_row = (unsigned)(scene_features[i].left_pos.y+0.5f);
        if(feature_row >= 0 && feature_row < result.size()){
            insertHintFeature(scene_features[i], result[feature_row]);
        }

        for(unsigned j = 1; j <= 2; j++){
            if(feature_row >= j && (feature_row-j) < result.size()){
                insertHintFeature(scene_features[i], result[feature_row-j]);
            }

            if(feature_row+j >= 0 && feature_row+j < result.size()){
                insertHintFeature(scene_features[i], result[feature_row+j]);
            }
        }
    }

    return result;
}

std::pair<float,float> findFeatureRegionRange(unsigned left_x, unsigned left_y,
                                              const std::vector<SIFTHintRow> &hint_rows){

    std::pair<float,float> result(5.0f, 512.0f-50.0f); // TODO: query screen width.
    std::list<std::pair<float,float> >::const_iterator it;
    return result;
    for(it = hint_rows[left_y].brackets.begin(); it != hint_rows[left_y].brackets.end(); ++it){
        if(it->first > left_x){
            result.second = it->second;
            break;
        }
        else{
            result.first = it->second;
        }
    }

    return result;
}

unsigned getImageCornerFeatures(IplImage *img, CvPoint2D32f *buffer, const unsigned max_features){
    IplImage *eigimage = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 1);
    IplImage *tempimage = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 1);
    int num_corners = max_features;

    IplImage *grey = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 1);
    cvCvtColor(img, grey, CV_BGR2GRAY);

    cvGoodFeaturesToTrack(grey, eigimage, tempimage, buffer, &num_corners, 0.05f, 2);

    cvFindCornerSubPix(grey, buffer, num_corners, cvSize(3,3), cvSize(-1,-1),
                       cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03));

    cvReleaseImage(&eigimage);
    cvReleaseImage(&tempimage);
    cvReleaseImage(&grey);

    return num_corners;
}

std::vector<SceneFeature> findCornerFeatures(IplImage *img_left, IplImage *img_right,
                                             const std::vector<StereoFeature> &stereo_features) {

    std::vector<SceneFeature> scene_features;
    for(unsigned i = 0; i < stereo_features.size(); i++){
        SceneFeature new_scene_feature;
        new_scene_feature.left_pos = Vector2D(stereo_features[i].feature_left.x,
                                              stereo_features[i].feature_left.y);
        new_scene_feature.right_pos = Vector2D(stereo_features[i].feature_right.x,
                                               stereo_features[i].feature_right.y);
        new_scene_feature.position = stereo_features[i].position;
        scene_features.push_back(new_scene_feature);
    }

    std::vector<SIFTHintRow> hint_rows = buildHintRows(scene_features);
    const unsigned MAX_FEATURES = 10000;

    CvPoint2D32f left_corners[MAX_FEATURES];
    CvPoint2D32f right_corners[MAX_FEATURES];

    unsigned num_left_corners = getImageCornerFeatures(img_left, left_corners, MAX_FEATURES);
    unsigned num_right_corners = getImageCornerFeatures(img_right, right_corners, MAX_FEATURES);

    std::vector<std::vector<CvPoint2D32f> > right_corners_scanlines;
    for(unsigned i = 0; i < img_left->height; i++){
        right_corners_scanlines.push_back(std::vector<CvPoint2D32f>());
    }

    for(unsigned i = 0; i < num_right_corners; i++){
        unsigned ypos = (unsigned)right_corners[i].y;
        if(ypos >= right_corners_scanlines.size()){
            continue;
        }

        if(right_corners[i].x > 5.0f && right_corners[i].x < (img_left->width-50) &&
           right_corners[i].y > 5.0f && right_corners[i].y < (img_left->height-5)){
            right_corners_scanlines[ypos].push_back(right_corners[i]);
        }
    }

    std::vector<SceneFeature> result;
    for(unsigned i = 0; i < num_left_corners; i++){
        if(left_corners[i].x <= 50.0f || left_corners[i].x >= (img_left->width-5) ||
           left_corners[i].y <= 5.0f || left_corners[i].y >= (img_left->height-5)){
            continue;
        }

        unsigned ypos = (unsigned)left_corners[i].y;
        unsigned start = 0, end = right_corners_scanlines.size();
        if(ypos > 1){ start = ypos - 1; }
        if(ypos < end-2){ end = ypos + 2; }


        Vector2D best_match_point;
        float least_dist = FLT_MAX;
        float least_dist2 = FLT_MAX;
        Vector2D left_pos(left_corners[i].x, left_corners[i].y);

        for(unsigned j = start; j < end; j++){
            for(unsigned k = 0; k < right_corners_scanlines[j].size(); k++){

                Vector2D right_pos(right_corners_scanlines[j][k].x, right_corners_scanlines[j][k].y);
                if(right_pos.x > left_pos.x){
                    continue;
                }

                float d = calculateRegionDiff(img_left, img_right, left_pos, right_pos, least_dist2);
                if(d < least_dist){
                    least_dist2 = least_dist;
                    least_dist = d;
                    best_match_point = right_pos;
                }
                else if(d < least_dist2){
                    least_dist2 = d;
                }
            }
        }

        std::pair<float,float> range = findFeatureRegionRange(left_pos.x, left_pos.y, hint_rows);

        if(least_dist < 0.15f && least_dist < 0.85f*least_dist2 &&
           best_match_point.x >= range.first && best_match_point.x <= range.second){
            SceneFeature new_scene_feature;
            new_scene_feature.position = Common::stereoProject(left_pos, best_match_point);
            new_scene_feature.left_pos = left_pos;
            new_scene_feature.right_pos = best_match_point;
            result.push_back(new_scene_feature);

            CvPoint lpos = cvPoint(left_pos.x, left_pos.y);
            //cvCircle(img_left, lpos, 3.0, CV_RGB(0, 0, 255), 1.0);

            CvPoint rpos = cvPoint(best_match_point.x, best_match_point.y);
           // cvCircle(img_right, rpos, 3.0, CV_RGB(0, 0, 255), 1.0);
        }
    }

    return result;
}

std::pair<std::vector<SceneFeature>,std::vector<SceneFeature> >
StereoFeatureCorrelation::findAllSceneFeatures(IplImage *img_left, IplImage *img_right,
                                               const std::vector<StereoFeature> &stereo_features) {

    std::pair<std::vector<SceneFeature>,std::vector<SceneFeature> > result;

    std::vector<SceneFeature> corner_features = findCornerFeatures(img_left, img_right, stereo_features);

    std::vector<SceneFeature> scene_features;
    for(unsigned i = 0; i < stereo_features.size(); i++){
        SceneFeature new_scene_feature;
        new_scene_feature.left_pos = Vector2D(stereo_features[i].feature_left.x,
                                              stereo_features[i].feature_left.y);
        new_scene_feature.right_pos = Vector2D(stereo_features[i].feature_right.x,
                                               stereo_features[i].feature_right.y);
        new_scene_feature.position = stereo_features[i].position;
        scene_features.push_back(new_scene_feature);
    }
    scene_features.insert(scene_features.end(), corner_features.begin(), corner_features.end());
    result.first = corner_features;

    std::vector<SIFTHintRow> hint_rows = buildHintRows(scene_features);

    IplImage *gray_left = Common::convertToGray32(img_left);
    IplImage *gray_right = Common::convertToGray32(img_right);

    IplImage *edge_left = cvCreateImage(cvSize(gray_left->width, gray_left->height), IPL_DEPTH_32F, 1);
    IplImage *edge_right = cvCreateImage(cvSize(gray_right->width, gray_right->height), IPL_DEPTH_32F, 1);

    cvSobel(gray_left, edge_left, 1, 0);
    cvSobel(gray_right, edge_right, 1, 0);

    cvReleaseImage(&gray_left);
    cvReleaseImage(&gray_right);


    const double edge_threshold = 0.25;
    const double edge_threshold2 = 0.2;

    for(unsigned y = 5; y < edge_left->height-5; y++){
        for(unsigned lx = 50; lx < edge_left->width-5; lx++){
            double lval;
            Common::getPixel(edge_left, lx, y, lval);
            lval = sqrt(lval*lval)/2.0;
            if(lval < edge_threshold){ continue; }

            float least_diff = FLT_MAX, least_diff2 = FLT_MAX;
            unsigned least_diff_index = 0;

            std::pair<float,float> range = findFeatureRegionRange(lx, y, hint_rows);
            for(unsigned rx = range.first; rx < std::min<float>(range.second,lx); rx++){
                double rval;
                Common::getPixel(edge_right, rx, y, rval);
                rval = sqrt(rval*rval)/2.0;
                if(rval < edge_threshold2){ continue; }

                float diff = calculateRegionDiff(img_left, img_right, Vector2D(lx, y), Vector2D(rx, y), least_diff2);
                if(diff < least_diff){
                    least_diff2 = least_diff;
                    least_diff = diff;
                    least_diff_index = rx;
                }
                else if(diff < least_diff2){
                    least_diff2 = diff;
                }
            }

            if(least_diff < 0.1f && least_diff < 0.8f*least_diff2){
                SceneFeature new_feature;
                new_feature.left_pos = Vector2D(lx, y);
                new_feature.right_pos = Vector2D(least_diff_index, y);
                new_feature.position = Common::stereoProject(new_feature.left_pos, new_feature.right_pos);
                result.second.push_back(new_feature);

                CvPoint lpos = cvPoint(lx, y);
                //cvCircle(img_left, lpos, 3.0, CV_RGB(0, 255, 0), 1.0);

                CvPoint rpos = cvPoint(least_diff_index, y);
                //cvCircle(img_right, rpos, 3.0, CV_RGB(0, 255, 0), 1.0);
            }
        }
    }

    cvReleaseImage(&edge_left);
    cvReleaseImage(&edge_right);

    return result;
}
