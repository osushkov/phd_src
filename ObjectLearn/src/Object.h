/*
 * Object.h
 *
 *  Created on: 30/10/2009
 *      Author: osushkov
 */

#ifndef _OBJECT_H_
#define _OBJECT_H_


#include "Features/FeatureMemory/ObjectSnapshotDB.h"
#include "Reconstruction/SuperQuadric.h"
#include "Visualisation/SceneRenderer.h"
#include "Visualisation/SuperQuadricRenderObject.h"
#include "Reconstruction/ReconstructionFrame.h"
#include "Reconstruction/BestFit.h"
#include "Reconstruction/SuperQuadricFit.h"
#include "ObjectSceneMatch.h"
#include "Mesh.h"
#include <string>

class ObjectShapeVerifier;

enum ObjectBuildStage {
    OBJECT_BS_NONE,
    OBJECT_BS_RAW_VIDEO,
    OBJECT_BS_SNAPSHOTS,
    OBJECT_BS_RAW_POINT_CLOUD,
    OBJECT_BS_COHERENT_POINT_CLOUD,
    OBJECT_BS_FULL
};


class Object {
  public:
    Object(std::string name);
    ~Object();

    std::string getObjectName(void) const;
    ObjectBuildStage getCurrentBuildStage(void) const;

    bool buildFull(void);
    bool buildStage(ObjectBuildStage stage);

    bool examine(void);

    bool sceneMatch(const std::vector<StereoFeature> &scene_features,
                    Transform &object_transform);

    bool sceneMatch(const std::vector<StereoFeature> &scene_features,
                    Transform &object_transform,
                    std::vector<unsigned> &matched_scene_features);

    void render(Transform transform);
    std::vector<ModelFrame> getModelFrames(void);

    Mesh getObjectMesh(void);
    SuperQuadric getSuperQuadric(void);

  private:
    const std::string object_name;

    std::string surround_video_path;
    std::string rotate_video_path;
    std::string snapshots_filename;
    std::string raw_point_cloud_filename;
    std::string coherent_point_cloud_filename;
    std::string shape_filename;

    ObjectBuildStage current_build_stage;
    ObjectSnapshotDB *snapshot_db;

    Vector3D last_position;
    Vector3D last_normal;
    bool have_last_position;

    SuperQuadric *object_superquadric;
    SuperQuadricRenderObject *render_superquadric;

    bool cull_by_normals;
    BestFit best_fit;

    ObjectSceneMatch *object_scene_match;

    bool loadObject(void);
    bool parseMainFile(void);
    bool writeMainFile(void);

    bool createObject(void);

    bool captureVideoData(void);
    bool learnObjectSnapshots(void);
    bool capturePointCloud(void);
    bool buildPointCloud(void);


    bool fitShapeToPointcloud(void);

    std::vector<Vector3D>
    normaliseFeaturePositions(const std::vector<Vector3D> &feature_positions, SuperQuadricFit fit);

    std::vector<ModelFrame>
    normaliseModelFrames(const std::vector<ModelFrame> &model_frames, SuperQuadricFit fit);

    std::vector<ModelFrame> readModelFrames(std::istream &in_stream);
    std::vector<Vector3D> getFeaturePositions(const std::vector<ModelFrame> &model_frames);
    void shiftToCOG(std::vector<Vector3D> &feature_positions,
                    std::vector<ModelFrame> &model_frames,
                    Vector3D cog);
    std::pair<float,float> calculatePositionsDistribution(const std::vector<Vector3D> &feature_positions);
    void filterFeaturePositionsByDistance(std::vector<Vector3D> &feature_positions,
                                          float avrg_dist, float threshold);
    void filterModelFramesByDistance(std::vector<ModelFrame> &model_frames,
                                     float avrg_dist, float threshold);
    bool moreFeaturesInRegion(const std::vector<Vector3D> &features, Vector3D rc, float rd, unsigned max);

    void centralisePoints(std::vector<Vector3D> &feature_positions,
                          std::vector<ModelFrame> &model_frames);

    std::pair<std::vector<SceneFeature>,std::vector<SceneFeature> >
    initialiseSilhouetteVerifier(ObjectShapeVerifier &verifier);



    std::string getObjectRootPath(void) const;

    bool readString(std::istream &in, std::string label, std::string &result);
    bool writeString(std::ostream &out, std::string label, std::string value);

    std::string buildStageToString(ObjectBuildStage stage);

};


#endif /* OBJECT_H_ */
