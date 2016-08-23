
#ifndef _ObjectLearner_H_
#define _ObjectLearner_H_

#include <vector>
#include <list>
#include <pthread.h>
#include <map>
#include <string>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "../Features/SIFTGenerator.h"
#include "../Features/SIFT/sift.h"
#include "../Features/SIFT/imgfeatures.h"
#include "../Features/StereoFeatureCorrelation.h"
#include "../Util/Semaphore.h"
#include "../Util/ReadWriteLock.h"

#include "SpacioTemporalLocality.h"
#include "TraceTracker.h"
#include "FrameHistoryBuffer.h"
#include "../CorrelatedFrames/CorrelatedFrameParser.h"
#include "../Features/FeatureMemory/ObjectSnapshotDB.h"

#include "../Visualisation/SceneRenderer.h"
#include "../Visualisation/PointCloudRenderObject.h"
#include "../Evaluation/MemoryEvaluator.h"


class ObjectLearner {
  public:

    ObjectLearner(std::string object_name);
    ~ObjectLearner();

    void learn(ObjectSnapshotDB* output_db);

    void newFrame(const std::vector<unsigned char> &left_frame,
                  const std::vector<unsigned char> &right_frame,
                  const std::vector<StereoFeature> &features,
                  const std::vector<float> &arm_joints);

    bool acceptingNewFrames(void){ return accepting_new_frames; }

  private:

    std::string object_name;

    unsigned current_frame;
    unsigned outstanding_frames;
    bool accepting_new_frames;

    TraceTracker trace_tracker;
    SpacioTemporalLocality locality;

    // This is a buffer of camera frames which is filled while the
    // arm is moving, and then used for computation purposes once
    // the arm stops moving. We dont put frames directly into the SIFT
    // generator while the arm is moving to prevent CPU contention and
    // dropped frames.
    std::vector<unsigned> buffered_frames;

    // this is only used for visualisation purposes.
    FrameHistoryBuffer frame_history;

    CorrelatedFrameParser correlated_frame_parser;

    ObjectSnapshotDB *snapshot_db;

    //MemoryEvaluator mem_eval;


    void doLearning(void);

    void filterFeaturesOnScale(std::vector<StereoFeature> &features, float max_scale);
    void filterFeaturesOnPosition(std::vector<StereoFeature> &features,
                                  const std::vector<float> &arm_joints);

    void calculateTrace(std::vector<StereoFeature> &features, unsigned cur_frame,
                        IplImage* left_frame, IplImage* right_frame);


    void localityFilter(std::vector<StereoFeature> &features,
                        std::vector<FeatureObjectType> &types);

    void combineFeatureLists(std::vector<StereoFeature> &features,
                             const std::vector<StereoFeature> &add);

    void cullSpacialOutliers(std::vector<StereoFeature> &features,
                             std::vector<FeatureObjectType> &feature_types);

    void cullFeatures(std::vector<StereoFeature> &target_features,
                      std::vector<StereoFeature> &existing_object_features,
                      const std::vector<StereoFeature> &arm_features);

    void renderTraces(const std::vector<Trace> &traces, IplImage* left_out, IplImage* right_out);

    void renderSceneFeatures(std::vector<SceneFeature> features,
                             IplImage *left_frame,
                             IplImage *right_frame,
                             CvScalar color);
    void renderStereoFeatures(std::vector<StereoFeature> features,
                              IplImage *left_frame,
                              IplImage *right_frame,
                              CvScalar color);

    std::vector<FeatureObjectType>
    calculateFeatureTypes(const std::vector<StereoFeature> &features);

    void renderFeatureTypes(const std::vector<StereoFeature> &features,
                            const std::vector<FeatureObjectType> &types,
                            IplImage *out);

    bool isValidFeaturePosition(const Vector3D &position, const std::vector<float> &arm_joints);

};

#endif

