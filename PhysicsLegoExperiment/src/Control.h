
#ifndef _Control_H_
#define _Control_H_

#include "Util/Vector3D.h"
#include "Util/Transform.h"
#include "Object/Object.h"
#include "Experiment/LightExperiment.h"
#include <vector>

namespace Control {
    //void initialise(void);
    //void perform(void);

	Transform locateObject(Object *obj, unsigned max_frames, unsigned &num_features_detected, bool &success);
    Transform pickUpObject(Object *obj, bool &success);

    Transform refineArmToObject(Object *obj);

    bool moveToExperiment(Object *obj, Transform object_pose, float floor_height,
                          LightExperiment *exp, float error_threshold);

    void moveToObjectExamineArmPose(unsigned pose_index);
	void moveArmToJoints(std::vector<float> joints);
    void moveArmOutOfTheWay(void);
    void moveArmToWorldPoint(Vector3D point, Vector3D approach);
    void moveArmToPoint(std::vector<Vector3D> pose);

    std::vector<Vector3D> gripperEulerAnglesToOrientation(std::vector<float> euler_angles);

    std::vector<std::vector<float> > getArmRootPoses(void);

    void safeMoveToJoints(std::vector<float> joints);

    void filterWorkspaceFeatures(std::vector<StereoFeature> &features);
};

#endif

