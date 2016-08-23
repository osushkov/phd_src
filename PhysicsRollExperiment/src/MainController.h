
#ifndef _MainController_H_
#define _MainController_H_

#include "Camera/Camera.h"
#include "Features/SIFTGenerator.h"
#include "Features/StereoFeatureCorrelation.h"

#include "Physics/PhysicsObject.h"
#include "Visualisation/SceneRenderer.h"
#include "Object/ObjectPhysModel.h"
#include "Experiment/Experiment.h"

#include <string>

class MainController {
  public:
    static MainController& instance(void);

    void initialise(void);
    void reload(void);

    void examineObject(std::string object_name);
    void sceneMatchObject(std::string object_name);
    
    void physicsTest(void);
    void experimentOnObject(std::string object_name);

    void openHand(float amount=1.0f, int speed=-1);
    void closeHand(float amount=1.0f, int speed=-1);
    void releaseHand(int speed=-1);

    void test(void);

    void mouseHandler(int event, int x, int y, int left_or_right);

  private:

    std::vector<ObjectPhysModel> createPossiblePhysModels(void);
    std::vector<Experiment*> createAvailableExperiments(Vector3D ground_plane_normal);

    MainController();
};

#endif

