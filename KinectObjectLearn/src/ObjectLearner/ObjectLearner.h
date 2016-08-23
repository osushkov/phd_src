
#ifndef _ObjectLearner_H_
#define _ObjectLearner_H_

#include <vector>
#include <list>
#include <pthread.h>
#include <map>
#include <string>
#include <cv.h>
#include <highgui.h>

#include "../KinectCamera/KinectCamera.h"
#include "../Features/SIFT/sift.h"
#include "../Features/SIFT/imgfeatures.h"
#include "../Features/SIFTFeature3D.h"
#include "../Util/Semaphore.h"
#include "../Util/ReadWriteLock.h"

#include "TraceTracker.h"
#include "FrameHistoryBuffer.h"
#include "../Features/FeatureMemory/ObjectSnapshotDB.h"

#include "../Visualisation/SceneRenderer.h"
#include "../Visualisation/PointCloudRenderObject.h"
#include "../Reconstruction/SnapshotFrames.h"


class ObjectLearner {
  public:

    ObjectLearner(std::string object_name);
    ~ObjectLearner();

    void learn(ObjectSnapshotDB* output_db);
    ObjectSnapshotDB* generateAugmentedDB(ObjectSnapshotDB* snapshot_db);

    void newFrame(const KinectCamera::CorrelatedImage frame,
                  const std::vector<SIFTFeature3D> features,
                  const std::vector<float> arm_joints);

    bool acceptingNewFrames(void){ return accepting_new_frames; }
    void saveSnapshotFrames();

  private:

    std::string object_name;

    unsigned current_frame;
    unsigned outstanding_frames;
    bool accepting_new_frames;

    TraceTracker trace_tracker;

    // This is a buffer of camera frames which is filled while the
    // arm is moving, and then used for computation purposes once
    // the arm stops moving. We dont put frames directly into the SIFT
    // generator while the arm is moving to prevent CPU contention and
    // dropped frames.
    std::vector<unsigned> buffered_frames;

    // this is only used for visualisation purposes.
    FrameHistoryBuffer frame_history;

    ObjectSnapshotDB *snapshot_db;
    SnapshotFrames snapshot_frames_manager;

    //MemoryEvaluator mem_eval;


    void doLearning(void);

    void filterFeaturesOnArmPosition(std::vector<SIFTFeature3D> &features,
                                     const std::vector<float> &arm_joints);
    void filterFeaturesOnWorldPosition(std::vector<SIFTFeature3D> &features);

    void calculateTrace(std::vector<SIFTFeature3D> &features, std::vector<float> arm_joints, 
                        unsigned cur_frame, KinectCamera::CorrelatedImage &frame);

    std::vector<bool> floodfillCorrelatedImage(KinectCamera::CorrelatedImage &frame, Vector2D scoord,
                                               const std::vector<bool> &excluded_region,
                                               Vector3D centre, float max_dist);

    void renderTraces(const std::vector<Trace> &traces, std::vector<SIFTFeature3D> &features, IplImage* out);

    void renderStereoFeatures(std::vector<SIFTFeature3D> features,
                              IplImage *out,
                              CvScalar color);

    std::vector<bool> calculateArmMask(const KinectCamera::CorrelatedImage &correlated_img, 
                                       const std::vector<SIFTFeature3D> &features,
                                       const std::vector<float> &arm_joints);

    void renderFeatureTypes(const std::vector<SIFTFeature3D> &features, IplImage *out);

    bool isValidFeaturePosition(const Vector3D &position, const std::vector<float> &arm_joints);
    bool isValidFeaturePosition(const Vector3D &position, const Vector3D gripper_pos);

    void augmentSnapshot(ObjectSnapshot *snapshot, std::vector<ObjectSnapshot*> all_snapshots);

};

#endif

