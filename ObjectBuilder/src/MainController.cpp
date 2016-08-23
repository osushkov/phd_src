
#include "MainController.h"

#include "Visualisation/SceneRenderer.h"
#include "Visualisation/BoxRenderObject.h"
#include "Visualisation/CylinderRenderObject.h"
#include "Visualisation/PointCloudRenderObject.h"
#include "Visualisation/TextureManager.h"
#include "Visualisation/SuperQuadricRenderObject.h"
#include "Settings.h"
#include "Util/Geometry.h"
#include "Util/Common.h"
#include "Features/StereoFeatureCorrelation.h"
#include "SuperQuadric.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <cstdio>
#include <cxcore.h>
#include <highgui.h>
#include <fstream>

std::vector<std::vector<Vector3D> > colors;

MainController& MainController::instance(void){
    static MainController main_controller;
    return main_controller;
}

MainController::MainController() {

}


void MainController::initialise(void){

}

std::vector<StereoFeature> generateSnapshot(RenderObject *obj, Transform transform){
    std::pair<IplImage*, IplImage*> img_pair = SceneRenderer::instance().renderObject(obj, transform);

    
    std::vector<feature> left_features, right_features;
    Common::extractFeatures(img_pair.first, left_features);
    Common::extractFeatures(img_pair.second, right_features);
    std::vector<StereoFeature> stereo_features = StereoFeatureCorrelation::correlateFeatures(left_features, right_features, 480);

    std::vector<Vector3D> scolors;
    for(unsigned i = 0; i < stereo_features.size(); i++){
        double r, g, b;
        Common::getPixel(img_pair.first, stereo_features[i].feature_left.x, stereo_features[i].feature_left.y, r, g, b);
        scolors.push_back(Vector3D(r/255.0f, g/255.0f, b/255.0f));
    }
    colors.push_back(scolors);

/*
    for(unsigned i = 0; i < stereo_features.size(); i++){
        cvCircle(img_pair.first, cvPoint(stereo_features[i].feature_left.x, stereo_features[i].feature_left.y), 2.0, CV_RGB(255, 0, 0), 1.0);
        cvCircle(img_pair.second, cvPoint(stereo_features[i].feature_right.x, stereo_features[i].feature_right.y), 2.0, CV_RGB(255, 0, 0), 1.0);
    }

    cvNamedWindow("LeftEye", 1);
    cvMoveWindow("LeftEye", 600, 30);

    cvNamedWindow("RightEye", 1);
    cvMoveWindow("RightEye", 1200, 30);

    cvShowImage("LeftEye", img_pair.first);
    cvShowImage("RightEye", img_pair.second);
    cvWaitKey();
*/

    return stereo_features;
    
}

std::vector<Vector3D> generateObjectOrientations(unsigned num){
    std::vector<Vector3D> sphere_points;

    float inc = (float)M_PI * (3.0f - sqrtf(5.0f));
    float off = 2.0f/num;
    for(unsigned i = 0; i < num; i++){
        float y = i * off - 1.0f + (off / 2.0f);
        float r = sqrtf(1.0f - y*y);
        float phi = i * inc;

        Vector3D new_point(cosf(phi)*r, y, sinf(phi)*r);
        new_point.normalise();
        sphere_points.push_back(new_point);
    }

    return sphere_points;
}

Matrix3 generateObjectOrientation(Vector3D ori){
    ori.normalise();

    Vector3D t[3] = { Vector3D(1.0f, 0.0f, 0.0), 
                      Vector3D(0.0f, 1.0f, 0.0f),
                      Vector3D(0.0f, 0.0f, 1.0f) };

    Vector3D v0, v1;
    bool have = false;

    for(unsigned i = 0; i < 3; i++){
        if(fabs(t[i].dotProduct(ori)) < 0.9f){
            v0 = ori.crossProduct(t[i]);
            have = true;
            break;
        }
    }

    if(!have){
        std::cout << "DONT HAVE!" << std::endl;
    }
    
    v0.normalise();
    v1 = v0.crossProduct(ori);
    v1.normalise();

    Matrix3 rmat;
    rmat(0,0) = v0.x;
    rmat(1,0) = v0.y;
    rmat(2,0) = v0.z;

    rmat(0,1) = v1.x;
    rmat(1,1) = v1.y;
    rmat(2,1) = v1.z;

    rmat(0,2) = ori.x;
    rmat(1,2) = ori.y;
    rmat(2,2) = ori.z;

    Matrix3 imat;
    imat.isTranspose(rmat);
    return imat;
}

RenderObject* generateObject(void){
    BoxRenderObject *box = new BoxRenderObject(5.25f*2.0f, 4.0f*2.0f, 5.5f*2.0f);

    char *texture_filenames[] = {
        "data/box1/side1.png",
        "data/box1/blank.png",
        "data/box1/blank.png",
        "data/box1/blank.png",
        "data/box1/blank.png",
        "data/box1/blank.png",
    };

    std::vector<unsigned> texture_ids;
    for(unsigned i = 0; i < 6; i++){
        IplImage *texture_img = cvLoadImage(texture_filenames[i]);

        for(int y = 0; y < texture_img->height; y++){
            for(int x = 0; x < texture_img->width; x++){
                double r, g, b;
                Common::getPixel(texture_img, x, y, r, g, b);
                Common::setPixel(texture_img, x, y, b, g, r);
            }
        }

        texture_ids.push_back(TextureManager::instance().loadTexture((unsigned char*)texture_img->imageData, texture_img->width, texture_img->height));
        cvReleaseImage(&texture_img);
    }

    box->setTextureIDs(texture_ids);
    box->setUseTexture(true);

    return box;
}

static void writeOutSnapshots(std::string filename, std::vector<std::pair<Vector3D,std::vector<StereoFeature> > > snapshots, SuperQuadric sq){
    std::ofstream out_file(filename.c_str(), std::ios::out|std::ios::binary);

    unsigned num_frames = snapshots.size();
    out_file.write((char*)&num_frames, sizeof(unsigned));
    
    for(unsigned i = 0; i < snapshots.size(); i++){
        out_file.write((char*)&snapshots[i].first, sizeof(Vector3D));

        unsigned num_features = snapshots[i].second.size();
        out_file.write((char*)&num_features, sizeof(unsigned));
        for(unsigned j = 0; j < snapshots[i].second.size(); j++){
            Common::writeFeature(out_file, snapshots[i].second[j]);
        }
    }

    sq.save(out_file);
}

void MainController::buildObject(std::string object_name){
    SuperQuadric sq(0.1f, 0.1f, 5.25f, 4.0f, 5.5f);
    
    SceneRenderer::instance().initialise();

    RenderObject *obj = generateObject();
    //SceneRenderer::instance().addObject(obj);
    //while(SceneRenderer::instance().renderScene());

    Transform cur_transform;
    cur_transform.shift = Vector3D(0.0f, 0.0f, -60.0f);
    cur_transform.mat.identity();

    unsigned num_snapshots = 200;
    std::vector<Vector3D> object_ori_vecs = generateObjectOrientations(num_snapshots);
    std::vector<std::pair<Vector3D,std::vector<StereoFeature> > > snapshots;
    for(unsigned i = 0; i < num_snapshots; i++){
        cur_transform.mat = generateObjectOrientation(object_ori_vecs[i]);
        std::vector<StereoFeature> snapshot_features = generateSnapshot(obj, cur_transform);

        Matrix3 inv_mat;
        inv_mat.isTranspose(cur_transform.mat);

        std::vector<StereoFeature> filtered_features;
        for(unsigned j = 0; j < snapshot_features.size(); j++){
            snapshot_features[j].position.z -= 60.0f;
            snapshot_features[j].position.x *= -1.0f;
            snapshot_features[j].position = inv_mat * snapshot_features[j].position;
            snapshot_features[j].position.x *= -1.0f;

            double d = sq.distance(snapshot_features[j].position);
            if(d < 0.5f){
                filtered_features.push_back(snapshot_features[j]);
            }
        }

        Vector3D view_dir = inv_mat * Vector3D(0.0f, 0.0f, 1.0f);

        snapshots.push_back(std::pair<Vector3D, std::vector<StereoFeature> >(view_dir, filtered_features));
        std::cout << "i: " << i << std::endl;
    }

    writeOutSnapshots("data/" + object_name + ".dat", snapshots, sq);

    PointCloudRenderObject *point_cloud = new PointCloudRenderObject(false, Vector3D(1.0f, 1.0f, 1.0f));
    std::vector<PointCloudPoint> points;

    for(unsigned i = 0; i < snapshots.size(); i++){
        for(unsigned j = 0; j < snapshots[i].second.size(); j++){
            PointCloudPoint new_point;
            new_point.pos = snapshots[i].second.at(j).position;
            new_point.color = colors[i].at(j);
            points.push_back(new_point);
        }
    }

    point_cloud->addPoints(points);
    SceneRenderer::instance().addObject(point_cloud);
    
    
    SuperQuadricRenderObject *sq_ro = new SuperQuadricRenderObject(sq);
    //sq_ro->setTransform(cur_transform);
    //SceneRenderer::instance().addObject(sq_ro);
    
    while(SceneRenderer::instance().renderScene());
    return;

/*
    SceneRenderer::instance().initialise();



    cvNamedWindow("LeftEye", 1);
    cvMoveWindow("LeftEye", 600, 30);

    cvNamedWindow("RightEye", 1);
    cvMoveWindow("RightEye", 950, 30);


    std::vector<StereoFeature> stereo_features = generateSnapshot(box, transform);
    std::pair<IplImage*, IplImage*> img_pair = SceneRenderer::instance().renderObject(box, transform);

    for(unsigned i = 0; i < stereo_features.size(); i++){
       CvScalar left_color = CV_RGB(0, 255, 0);
        CvScalar right_color = CV_RGB(0, 255, 0);
        CvPoint pos = cvPoint(stereo_features[i].feature_left.x, stereo_features[i].feature_left.y);
        cvCircle(img_pair.first, pos, 2.0, CV_RGB(0, 255, 0), 1.0);

        pos = cvPoint(stereo_features[i].feature_right.x, stereo_features[i].feature_right.y);
        cvCircle(img_pair.second, pos, 2.0, CV_RGB(0, 255, 0), 1.0);
    }

    cvShowImage("LeftEye", img_pair.first);
    cvShowImage("RightEye", img_pair.second);

    char key = cvWaitKey();

    cvReleaseImage(&img_pair.first);
    cvReleaseImage(&img_pair.second);
    */
}
