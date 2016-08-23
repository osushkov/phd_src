/*
 * Object.h
 *
 *  Created on: 30/10/2009
 *      Author: osushkov
 */

#ifndef _OBJECT_H_
#define _OBJECT_H_


#include "../Features/FeatureMemory/ObjectSnapshotDB.h"
#include "../Visualisation/SuperQuadricRenderObject.h"
#include "../ObjectSceneMatch.h"
#include <string>


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

    bool examine(void);

    bool sceneMatch(const std::vector<StereoFeature> &scene_features,
                    Transform &object_transform,
                    unsigned &num_features_matched);

    void resetSceneMatch(void);

    void render(Transform transform);
    std::vector<ModelFrame> getModelFrames(void);

    // Gets the 3 primary axes of the object. The first is the longest axis, the 2nd is the second
    // longest axis perpendicular to the first, the 3rd is the shortest axis perpendicular to the
    // other 2.
    std::vector<Vector3D> getPrimaryAxes(void);

    double findGripSize(Vector3D gripperA, Vector3D gripperB);

    Transform getObjectTransform(void);
    void setObjectTransform(Transform new_transform);

    void invalidateTransform(void);
    bool isTransformValid(void);

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

    SuperQuadric *object_superquadric;
    SuperQuadricRenderObject *render_superquadric;

    bool cull_by_normals;

    ObjectSceneMatch *object_scene_match;

    Transform object_transform;
    bool transform_is_valid;

    bool loadObject(void);
    bool parseMainFile(void);
    bool writeMainFile(void);


    std::string getObjectRootPath(void) const;

    bool readString(std::istream &in, std::string label, std::string &result);
    bool writeString(std::ostream &out, std::string label, std::string value);

    std::string buildStageToString(ObjectBuildStage stage);

};


#endif /* OBJECT_H_ */
