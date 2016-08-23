/*
 * ObjectShapeVerifier.h
 *
 *  Created on: 17/03/2010
 *      Author: osushkov
 */

#ifndef OBJECTSHAPEVERIFIER_H_
#define OBJECTSHAPEVERIFIER_H_

#include "../Reconstruction/SuperQuadric.h"
#include "../Util/Transform.h"
#include "../Util/Vector3D.h"
#include "../Util/Vector2D.h"
#include "../Util/ParallelServer.h"
#include "../Features/StereoFeature.h"
#include "../Features/StereoFeatureCorrelation.h"
#include "../Object.h"
#include "../Visualisation/SuperQuadricRenderObject.h"

#include <opencv/cxcore.h>
#include <map>

struct ShapeVerifierData {
    std::pair<std::vector<SceneFeature>,std::vector<SceneFeature> > scene_features;
    std::vector<SceneFeature> background_scene_features;
    Transform transform;

    IplImage *img_left, *img_right;
};


class ObjectShapeVerifier {
  public:
    ObjectShapeVerifier();
    ~ObjectShapeVerifier();

    static std::vector<std::pair<std::string,std::string> > getSilhouetteImages(std::string path,
                                                                                unsigned num);

    std::pair<std::vector<SceneFeature>,std::vector<SceneFeature> >
    initialise(std::vector<std::pair<IplImage*,IplImage*> > img_pairs,
               std::vector<std::vector<StereoFeature> > img_features,
               Object *object);

    float calculateModelSceneFeatureError(SuperQuadric sq, Transform t, int ransac_id);
    float numMatchingSceneFeatures(SuperQuadric sq, Transform t, float max_error);

    float calculateModelFeatureOverlapPenalty(SuperQuadric sq, Transform t);
    float calculateModelFeatureOverlapSubPenalty(std::vector<Vector3D> surface_points, Transform t, ShapeVerifierData &sub_data);

    void setDebug(bool val){ debug = val; }

  private:

    bool debug;
    ParallelServer pserver;
    std::vector<ShapeVerifierData> data;
    std::vector<ShapeVerifierData> all_data;
    float cam_width, cam_height;
    float cam_theta, cam_phi;
    Vector2D cam_lrx, cam_tby;


    float sceneFeaturesError(const std::vector<SceneFeature> &features,
                             const SuperQuadric &sq,
                             Vector3D shift,
                             Matrix3 inv_mat,
                             Transform t);

    unsigned numMatchingSceneFeaturesFrame(const std::vector<SceneFeature> &features,
                                           const SuperQuadric &sq,
                                           Vector3D shift,
                                           Matrix3 inv_mat,
                                           Transform t,
                                           float max_error);

    void getObjectSilhouette(std::vector<Vector3D> surface_points,
                             Transform transform,
                             std::vector<std::pair<Vector2D,float> > &left_cam_pixels,
                             std::vector<std::pair<Vector2D,float> > &right_cam_pixels,
                             float scale_factor);

    void silhouettePixelsFromHull(const std::vector<Vector2D> &hull,
                                  const unsigned pixel_gap,
                                  std::vector<std::pair<Vector2D,float> > &pixels);

    std::pair<std::vector<SceneFeature>,std::vector<SceneFeature> >
    generateData(std::vector<std::pair<IplImage*,IplImage*> > img_pairs,
                 std::vector<std::vector<StereoFeature> > img_features,
                 Object *object);

    void generateDataPixelBuckets(ShapeVerifierData &data, IplImage *edge_image, bool left);

    void blankOutEdges(std::pair<IplImage*,IplImage*> edge_pair,
                       std::vector<StereoFeature> img_features,
                       std::vector<unsigned> matched_features,
                       float threshold);

    void blankSceneFeatures(std::vector<SceneFeature> &scene_features,
                            const std::vector<StereoFeature> &img_features,
                            const std::vector<unsigned> &matched_features,
                            Vector3D object_pos,
                            float threshold1,
                            float threshold2);

    void blankOutInternalEdges(IplImage* edge_image, const std::vector<Vector2D> &silhouette_pixels);
    void blankOutBackgroundEdges(std::pair<IplImage*,IplImage*> edge_pair,
                                 const std::vector<SceneFeature> &features,
                                 Vector3D obj_pos, float dist_threshold);

    void blankOutForegroundFeatures(std::vector<SceneFeature> &features,
                                    Vector3D object_pos,
                                    float threshold);


    Vector2D projectPointToCamera(Vector3D cam_pos, Vector3D point_pos);
    Vector2D pointWorldPosToPixelPos(Vector2D point_pos);

    float compareImageToSilhouette(ShapeVerifierData &data,
                                   std::vector<std::pair<Vector2D,float> > &silhouette,
                                   bool left);

    std::vector<float> applySobelFilter(IplImage *img, bool vertical);
    float gradientDiffScore(float g0, float g1);

};

#endif /* OBJECTSHAPEVERIFIER_H_ */
