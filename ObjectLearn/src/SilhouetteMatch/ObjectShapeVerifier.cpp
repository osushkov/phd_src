/*
 * ObjectShapeVerifier.cpp
 *
 *  Created on: 17/03/2010
 *      Author: osushkov
 */

#include "ObjectShapeVerifier.h"

#include "../Util/Geometry.h"
#include "../Util/ConvexHull.h"
#include "../Util/ParallelServer.h"
#include "../Util/Timer.h"

#include "../Settings.h"
#include "../Mesh.h"
#include "../OptimiserClass/Optimiser.h"
#include "../OptimiserClass/NelderMead.h"
#include "../OptimiserClass/ParticleSwarm.h"
#include "../Visualisation/MeshRenderObject.h"
#include "../Visualisation/SceneRenderer.h"

#include <cmath>
#include <cassert>
#include <fstream>
#include <algorithm>

#define SCALE_FACTOR (1.0f)

ObjectShapeVerifier::ObjectShapeVerifier() : pserver(PSM_PARALLEL) {
    debug = false;

    cam_width = Settings::instance().getIntValue("camera", "res_width");
    cam_height = Settings::instance().getIntValue("camera", "res_height");

    cam_theta =  Settings::instance().getFloatValue("camera", "xfov")/2.0f;
    cam_phi = Settings::instance().getFloatValue("camera", "yfov")/2.0f;

    cam_lrx = Vector2D(-tanf(cam_theta*M_PI/180.0f), tanf(cam_theta*M_PI/180.0f));
    cam_tby = Vector2D(tanf(cam_phi*M_PI/180.0f), -tanf(cam_phi*M_PI/180.0f));
}

ObjectShapeVerifier::~ObjectShapeVerifier(){

}

std::vector<std::pair<std::string,std::string> > ObjectShapeVerifier::getSilhouetteImages(std::string path,
                                                                                          unsigned num){
    std::string filename = path + "record_file.txt";
    std::ifstream file_stream(filename.c_str());

    std::string tmp_string;
    int tmp_int;
    float tmp_float;

    file_stream >> tmp_int;
    file_stream >> tmp_int;

    file_stream >> tmp_string;
    file_stream >> tmp_int;

    std::vector<std::pair<std::string,std::string> > all_filenames;
    while(!file_stream.eof()){
        file_stream >> tmp_string;
        if(tmp_string.size() == 0){ break; }
        file_stream >> tmp_int;

        std::string left_filename, right_filename;
        file_stream >> left_filename;
        file_stream >> right_filename;
        if(left_filename.size() == 0 || right_filename.size() == 0){ break; }
        all_filenames.push_back(std::pair<std::string,std::string>(left_filename, right_filename));

        file_stream >> tmp_float;
        file_stream >> tmp_int;
        for(unsigned i = 0; i < 6; i++){
            file_stream >> tmp_float;
        }
    }

    std::vector<std::pair<std::string,std::string> > result;
    for(unsigned i = 0; i < num; i++){
        result.push_back(all_filenames[rand()%all_filenames.size()]);
    }

    return result;
}

std::pair<std::vector<SceneFeature>,std::vector<SceneFeature> >
ObjectShapeVerifier::initialise(std::vector<std::pair<IplImage*,IplImage*> > img_pairs,
                                std::vector<std::vector<StereoFeature> > img_features,
                                Object *object){
    assert(object != NULL);
    assert(img_features.size() == img_pairs.size());

    return generateData(img_pairs, img_features, object);
}


struct ObjectShapeVerifierWorkerData {

    ObjectShapeVerifierWorkerData(ObjectShapeVerifier *shape_verifier,
                                  std::vector<ShapeVerifierData> &all_data,
                                  std::vector<Vector3D> &surface_points,
                                  Transform transform,
                                  Util::Semaphore &lock,
                                  float &result) :
        shape_verifier(shape_verifier), all_data(all_data), surface_points(surface_points),
        transform(transform), lock(lock), result(result) {}

    ObjectShapeVerifier *shape_verifier;
    std::vector<ShapeVerifierData> &all_data;
    std::vector<Vector3D> &surface_points;
    Transform transform;
    Util::Semaphore &lock;
    float &result;
};

class ObjectShapeVerifierWorker : public ParallelExecutor {
  public:

    ObjectShapeVerifierWorker(){};
    ~ObjectShapeVerifierWorker(){};

    void performTask(void *task_data, unsigned rank, unsigned size){

        ObjectShapeVerifierWorkerData *data = (ObjectShapeVerifierWorkerData*)task_data;
        float local_result = 0.0f;

        unsigned start = (rank*data->all_data.size())/size;
        unsigned end = ((rank+1)*data->all_data.size())/size;

        for(unsigned i = start; i < end; i++){
            local_result += data->shape_verifier->calculateModelFeatureOverlapSubPenalty(data->surface_points, data->transform, data->all_data[i]);
        }

        data->lock.wait();
        data->result += local_result;
        data->lock.signal();

    }
};

float ObjectShapeVerifier::calculateModelFeatureOverlapPenalty(SuperQuadric sq, Transform t){
    float result = 0.0f;
    std::vector<Vector3D> surface_points = sq.getSurfacePoints(M_PI/8.0f);

    if(debug){
        for(unsigned i = 0; i < all_data.size(); i++){
            result += calculateModelFeatureOverlapSubPenalty(surface_points, t, all_data[i]);
        }
    }
    else{

        Util::Semaphore lock(1);
        Util::Semaphore task_sem;
        ObjectShapeVerifierWorker worker_thread;

        ObjectShapeVerifierWorkerData *task_data =
            new ObjectShapeVerifierWorkerData(this, all_data, surface_points, t, lock, result);

        ParallelTask task(&worker_thread, task_data, &task_sem);
        pserver.executeParallelTask(task);
        task_sem.wait();

        delete task_data;

    }

    assert(all_data.size() > 0);
    return result/all_data.size();
}

float ObjectShapeVerifier::calculateModelFeatureOverlapSubPenalty(std::vector<Vector3D> surface_points, Transform t, ShapeVerifierData &sub_data){


    Matrix3 tmp_mat = t.mat;
    tmp_mat.isInverse(t.mat);

    Transform altered_transform = sub_data.transform;
    altered_transform.mat = altered_transform.mat*tmp_mat; // IS THIS ORDER CORRECT? TODO
    altered_transform.quaternions = Common::matrixToQuaternions(altered_transform.mat);
    altered_transform.secondary_shift = t.shift;

    IplImage *img_left, *img_right;
    if(debug){
        cvNamedWindow("LeftTemp");
        cvNamedWindow("RightTemp");

        img_left = sub_data.img_left;//cvCreateImage(cvGetSize(sub_data.edge_image_pair.first), IPL_DEPTH_8U, 3);
        img_right = sub_data.img_right;//cvCreateImage(cvGetSize(sub_data.edge_image_pair.first), IPL_DEPTH_8U, 3);

        //cvCvtColor(sub_data.edge_image_pair.first, img_left, CV_GRAY2BGR);
        //cvCvtColor(sub_data.edge_image_pair.second, img_right, CV_GRAY2BGR);
    }

    std::vector<std::pair<Vector2D,float> > left_silhouette, right_silhouette;
    getObjectSilhouette(surface_points, altered_transform, left_silhouette, right_silhouette, SCALE_FACTOR);

    std::vector<Vector2D> left_hull, right_hull;
    for(unsigned i = 0; i < left_silhouette.size(); i++){
        left_hull.push_back(left_silhouette[i].first);
    }
    for(unsigned i = 0; i < right_silhouette.size(); i++){
        right_hull.push_back(right_silhouette[i].first);
    }

    float left_hull_size = 0.0f;
    float right_hull_size = 0.0f;

    for(unsigned i = 0; i < left_hull.size(); i++){
        for(unsigned j = i+1; j < left_hull.size(); j++){
            float d = (left_hull[i] - left_hull[j]).length();
            if(d > left_hull_size){
                left_hull_size = d;
            }
        }
    }

    for(unsigned i = 0; i < right_hull.size(); i++){
        for(unsigned j = i+1; j < right_hull.size(); j++){
            float d = (right_hull[i] - right_hull[j]).length();
            if(d > right_hull_size){
                right_hull_size = d;
            }
        }
    }


    if(debug){
        for(unsigned i = 0; i < left_hull.size(); i++){
            unsigned next = (i+1)%left_hull.size();
            CvPoint spos = cvPoint(left_hull[i].x, left_hull[i].y);
            CvPoint epos = cvPoint(left_hull[next].x, left_hull[next].y);

            if(spos.x > 0 && spos.x < img_left->width &&
               spos.y > 0 && spos.y < img_left->height &&
               epos.x > 0 && epos.x < img_left->width &&
               epos.y > 0 && epos.y < img_left->height){
                cvLine(img_left, spos, epos, CV_RGB(0, 0, 255));
            }
        }

        for(unsigned i = 0; i < right_hull.size(); i++){
            unsigned next = (i+1)%right_hull.size();
            CvPoint spos = cvPoint(right_hull[i].x, right_hull[i].y);
            CvPoint epos = cvPoint(right_hull[next].x, right_hull[next].y);

            if(spos.x > 0 && spos.x < img_left->width &&
               spos.y > 0 && spos.y < img_left->height &&
               epos.x > 0 && epos.x < img_left->width &&
               epos.y > 0 && epos.y < img_left->height){
                cvLine(img_right, spos, epos, CV_RGB(0, 0, 255));
            }
        }
    }


    float obj_dist = (altered_transform.mat*(altered_transform.secondary_shift) + altered_transform.shift).length();
    float result = 0.0f;

    std::vector<SceneFeature> all_scene_features = sub_data.background_scene_features;
    for(unsigned i = 0; i < all_scene_features.size(); i++){
        float fdist = all_scene_features[i].position.length();
        if(fdist > obj_dist+10.0f && // TODO: define the max size of the object
           (pointInConvexHull(all_scene_features[i].left_pos, left_hull) ||
           pointInConvexHull(all_scene_features[i].right_pos, right_hull))){

            float error = pointDistanceFromHull(all_scene_features[i].left_pos, left_hull, 2)/left_hull_size +
                          pointDistanceFromHull(all_scene_features[i].right_pos, right_hull, 2)/right_hull_size;
            result += error;

            if(debug){
                CvPoint lpos = cvPoint(all_scene_features[i].left_pos.x, all_scene_features[i].left_pos.y);
                cvCircle(img_left, lpos, 3.0, CV_RGB(255, 0, 0), 1.0);

                CvPoint rpos = cvPoint(all_scene_features[i].right_pos.x, all_scene_features[i].right_pos.y);
                cvCircle(img_right, rpos, 3.0, CV_RGB(255, 0, 0), 1.0);
            }

        }
        else if(debug){

            CvPoint lpos = cvPoint(all_scene_features[i].left_pos.x, all_scene_features[i].left_pos.y);
            cvCircle(img_left, lpos, 3.0, CV_RGB(0, 255, 0), 1.0);

            CvPoint rpos = cvPoint(all_scene_features[i].right_pos.x, all_scene_features[i].right_pos.y);
            cvCircle(img_right, rpos, 3.0, CV_RGB(0, 255, 0), 1.0);

        }
    }

    if(debug){
        cvShowImage("LeftTemp", img_left);
        cvShowImage("RightTemp", img_right);
        cvWaitKey();

        //cvReleaseImage(&img_left);
        //cvReleaseImage(&img_right);
    }

    return result;
}

void ObjectShapeVerifier::getObjectSilhouette(std::vector<Vector3D> surface_points,
                                              Transform transform,
                                              std::vector<std::pair<Vector2D,float> > &left_cam_pixels,
                                              std::vector<std::pair<Vector2D,float> > &right_cam_pixels,
                                              float scale_factor){

    std::vector<Vector2D> left_projected_verts;
    std::vector<Vector2D> right_projected_verts;

    Util::Timer timer0, timer1, timer2;

    timer0.start();
    for(unsigned i = 0; i < surface_points.size(); i++){
        Vector3D t = transform.mat*(surface_points[i] + transform.secondary_shift) + transform.shift;
        Vector2D left_pos = projectPointToCamera(Vector3D(-6.0f, 0.0f, 0.0f), t);
        Vector2D right_pos = projectPointToCamera(Vector3D(6.0f, 0.0f, 0.0f), t);
        left_projected_verts.push_back(pointWorldPosToPixelPos(left_pos));
        right_projected_verts.push_back(pointWorldPosToPixelPos(right_pos));
    }
    timer0.stop();


    timer1.start();
    std::vector<Vector2D> left_hull = convexHull(left_projected_verts);
    std::vector<Vector2D> right_hull = convexHull(right_projected_verts);
    timer1.stop();

    left_cam_pixels.clear();
    right_cam_pixels.clear();

    timer2.start();

    for(unsigned i = 0; i < left_hull.size(); i++){
        left_cam_pixels.push_back(std::pair<Vector2D,float>(left_hull[i],0.0f));
    }

    for(unsigned i = 0; i < right_hull.size(); i++){
        right_cam_pixels.push_back(std::pair<Vector2D,float>(right_hull[i],0.0f));
    }

    //silhouettePixelsFromHull(left_hull, 10, left_cam_pixels);
    //silhouettePixelsFromHull(right_hull, 10, right_cam_pixels);

    timer2.stop();


/*
    std::cout << timer0.getNumElapsedSeconds()*1000.0f << " "
              << timer1.getNumElapsedSeconds()*1000.0f << " "
              << timer2.getNumElapsedSeconds()*1000.0f << " "
              << timer3.getNumElapsedSeconds()*1000.0f << "!" << std::endl;
*/
}

void ObjectShapeVerifier::silhouettePixelsFromHull(const std::vector<Vector2D> &hull,
                                                   const unsigned pixel_gap,
                                                   std::vector<std::pair<Vector2D,float> > &pixels){
    for(unsigned i = 0; i < hull.size(); i++){
        unsigned next_index = (i+1)%hull.size();
        std::vector<Vector2D> segment_pixels =
            Geometry::getLinePixels(hull[i], hull[next_index], pixel_gap);

        Vector2D line_vec = hull[next_index] - hull[i];
        line_vec.normalise();
        line_vec.rotate(90.0f * M_PI/180.0f);

        float gradient = atan2f(line_vec.y, line_vec.x);
        for(unsigned j = 0; j < segment_pixels.size(); j++){
            pixels.push_back(std::pair<Vector2D,float>(segment_pixels[j], gradient));
        }
    }
}

struct ObjectSceneMatchWorkerData {
    ObjectSceneMatchWorkerData(IplImage *img_left,
                               IplImage *img_right,
                               std::vector<StereoFeature> &features,
                               Object *object) :
        img_left(img_left), img_right(img_right), features(features), object(object) {
        assert(object != NULL);
    }

    IplImage *img_left, *img_right;
    std::vector<StereoFeature> &features;
    Object *object;

    Transform result_transform;
    bool success;
    std::vector<unsigned> matched_features;
    std::pair<std::vector<SceneFeature>,std::vector<SceneFeature> > scene_features;
};

class ObjectSceneMatchWorker : public ParallelExecutor {
  public:

    ObjectSceneMatchWorker(){};
    ~ObjectSceneMatchWorker(){};

    void performTask(void *task_data, unsigned rank, unsigned size){
        ObjectSceneMatchWorkerData *data = (ObjectSceneMatchWorkerData*)task_data;

        data->success = data->object->sceneMatch(data->features,
                                                 data->result_transform,
                                                 data->matched_features);

        data->scene_features = StereoFeatureCorrelation::findAllSceneFeatures(data->img_left,
                                                                              data->img_right,
                                                                              data->features);
    }
};

std::pair<std::vector<SceneFeature>,std::vector<SceneFeature> >
ObjectShapeVerifier::generateData(std::vector<std::pair<IplImage*,IplImage*> > img_pairs,
                                  std::vector<std::vector<StereoFeature> > img_features,
                                  Object *object){

    ParallelServer pserver(PSM_PIPELINE);
    ObjectSceneMatchWorker worker_thread;
    Util::Semaphore task_sem;

    ObjectSnapshotDB *arm_db = ObjectSnapshotDB::getArmSnapshotDB();

    std::vector<ObjectSceneMatchWorkerData*> match_data;
    for(unsigned i = 0; i < img_features.size(); i++){
        ObjectSceneMatchWorkerData *data = new ObjectSceneMatchWorkerData(img_pairs[i].first,
                                                                          img_pairs[i].second,
                                                                          img_features[i],
                                                                          object);
        ParallelTask task(&worker_thread, data, &task_sem);
        pserver.executeParallelTask(task);
        match_data.push_back(data);
    }

    for(unsigned i = 0; i < match_data.size(); i++){
        task_sem.wait();
    }

    std::pair<std::vector<SceneFeature>,std::vector<SceneFeature> > result;
    for(unsigned i = 0; i < match_data.size(); i++){
        ShapeVerifierData new_data;

        //if(object->sceneMatch(img_features[i], new_data.transform, matched_scene_features)){
        if(match_data[i]->success){
            bool is_reliable = true;
            new_data.transform = match_data[i]->result_transform;

            SceneMatchResult arm_match_result = arm_db->matchScene(img_features[i], false);
            std::vector<unsigned> matched_arm_features;
            std::set<unsigned>::iterator it;
            for(it = arm_match_result.matched_scene_features.begin();
                it != arm_match_result.matched_scene_features.end();
                ++it){
                matched_arm_features.push_back(*it);
            }


            // TODO: this sux
            if(matched_arm_features.size() < 7){
                is_reliable = false;
            }
            std::cout << "matched arm features: " << matched_arm_features.size() << std::endl;


            new_data.scene_features = match_data[i]->scene_features;

            std::vector<SceneFeature> all_scene_features = new_data.scene_features.first;
            all_scene_features.insert(all_scene_features.end(),
                                      new_data.scene_features.second.begin(),
                                      new_data.scene_features.second.end());

            for(unsigned j = 0; j < img_features[i].size(); j++){
                SceneFeature new_scene_feature;
                new_scene_feature.position = img_features[i].at(j).position;
                all_scene_features.push_back(new_scene_feature);
            }

            new_data.background_scene_features = all_scene_features;
            blankOutForegroundFeatures(new_data.background_scene_features, new_data.transform.shift, 10.0f);

            blankSceneFeatures(new_data.scene_features.first, img_features[i], matched_arm_features,
                               new_data.transform.shift, 7.0f, 10.0f);
            blankSceneFeatures(new_data.scene_features.second, img_features[i], matched_arm_features,
                               new_data.transform.shift, 7.0f, 10.0f);

            Matrix3 inv_mat;
            inv_mat.isInverse(new_data.transform.mat);

            if(is_reliable && i != 1){
                for(unsigned j = 0; j < new_data.scene_features.first.size(); j++){
                    SceneFeature new_feature = new_data.scene_features.first.at(j);
                    new_feature.position = inv_mat*(new_feature.position - new_data.transform.shift);
                    result.first.push_back(new_feature);
                }
                for(unsigned j = 0; j < new_data.scene_features.second.size(); j++){
                    SceneFeature new_feature = new_data.scene_features.second.at(j);
                    new_feature.position = inv_mat*(new_feature.position - new_data.transform.shift);
                    result.second.push_back(new_feature);
                }
            }

            new_data.img_left = match_data[i]->img_left;
            new_data.img_right = match_data[i]->img_right;
/*
            cvNamedWindow("Temp");

            for(unsigned j = 0; j < new_data.scene_features.first.size(); j++){
                CvPoint rpos = cvPoint(new_data.scene_features.first[j].left_pos.x, new_data.scene_features.first[j].left_pos.y);
                cvCircle(new_data.img_left, rpos, 3.0, CV_RGB(255, 0, 0), 1.0);
            }

            for(unsigned j = 0; j < new_data.scene_features.second.size(); j++){
                CvPoint rpos = cvPoint(new_data.scene_features.second[j].left_pos.x, new_data.scene_features.second[j].left_pos.y);
                cvCircle(new_data.img_left, rpos, 3.0, CV_RGB(255, 0, 0), 1.0);
            }

            for(unsigned j = 0; j < matched_arm_features.size(); j++){
                CvPoint rpos = cvPoint(img_features[i][matched_arm_features[j]].feature_left.x, img_features[i][matched_arm_features[j]].feature_left.y);
                cvCircle(new_data.img_left, rpos, 3.0, CV_RGB(0, 0, 255), 1.0);
            }

            cvShowImage("Temp", new_data.img_left);
            cvWaitKey();
*/



            if(is_reliable){
                data.push_back(new_data);
            }
            all_data.push_back(new_data);
        }
    }

    for(unsigned i = 0; i < match_data.size(); i++){
        delete match_data[i];
    }

    return result;
}

void ObjectShapeVerifier::blankSceneFeatures(std::vector<SceneFeature> &scene_features,
                                             const std::vector<StereoFeature> &img_features,
                                             const std::vector<unsigned> &matched_features,
                                             Vector3D object_pos,
                                             float threshold1,
                                             float threshold2){

    std::vector<SceneFeature> result_scene_features;
    for(unsigned i = 0; i < scene_features.size(); i++){
        if((scene_features[i].position-object_pos).length() > threshold2){
            continue;
        }

        bool is_valid = true;
        for(unsigned j = 0; j < matched_features.size(); j++){
            if((scene_features[i].position - img_features[matched_features[j]].position).length() < threshold1){
                is_valid = false;
                break;
            }
        }

        if(is_valid){
            result_scene_features.push_back(scene_features[i]);
        }
    }

/*
    cvNamedWindow("Temp");
    IplImage *tmp = cvCreateImage(cvSize(512, 384), IPL_DEPTH_8U, 3);
    cvSet(tmp, CV_RGB(0,0,0));

    for(unsigned i = 0; i < result_scene_features.size(); i++){
        CvPoint rpos = cvPoint(result_scene_features[i].left_pos.x, result_scene_features[i].left_pos.y);
        cvCircle(tmp, rpos, 3.0, CV_RGB(255, 0, 0), 1.0);
    }

    cvShowImage("Temp", tmp);
    cvWaitKey();
    cvReleaseImage(&tmp);

    std::cout << "SIZE! " << result_scene_features.size() << std::endl;
    */
    scene_features = result_scene_features;
}


void ObjectShapeVerifier::blankOutForegroundFeatures(std::vector<SceneFeature> &features,
                                                     Vector3D object_pos,
                                                     float threshold){

    std::vector<SceneFeature> new_features;
    for(unsigned i = 0; i < features.size(); i++){
        float dist = (object_pos - features[i].position).length();
        if(dist > threshold){
            new_features.push_back(features[i]);
        }
    }

    features = new_features;
}

Vector2D ObjectShapeVerifier::projectPointToCamera(Vector3D cam_pos, Vector3D point_pos){
    float plane_intersect_t = 0.0f;
    Vector3D cam_view_dir(0.0f, 1.0f, 0.0f);
    Geometry::linePlaneIntersect(point_pos, cam_pos, cam_view_dir, 1.0f, plane_intersect_t);
    Vector3D result_3d = point_pos + plane_intersect_t*(cam_pos-point_pos);
    return Vector2D(result_3d.x-cam_pos.x, result_3d.z-cam_pos.z);
}

Vector2D ObjectShapeVerifier::pointWorldPosToPixelPos(Vector2D point_pos){
    float xr = (point_pos.x - cam_lrx.x)/(cam_lrx.y - cam_lrx.x);
    float yr = (point_pos.y - cam_tby.x)/(cam_tby.y - cam_tby.x);

    return Vector2D(cam_width - xr*cam_width, cam_height - yr*cam_height);
}

