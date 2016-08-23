
#ifndef _MainController_H_
#define _MainController_H_

#include "KinectCamera/KinectCamera.h"
#include "Features/SIFTFeature3D.h"

#include <string>

class MainController {
  public:
    static MainController& instance(void);

    void initialise(void);
    void reload(void);

    void learnObject(std::string object_name);
    void examineObject(std::string object_name);

    void generateSIFT(std::string vid_path, std::string out_path);
    void reconstruct(std::string object_name);
    void test(void);
    void generateMovement(std::string record_path);
    void rotate(std::string record_path);
    void sceneMatchObject(std::string object_name);

    void stitchTogether(std::string object0, std::string object1);
    void fitShape(std::string object_name);
    void evalMatch(std::string object_name);
    void finalisePointCloud(std::string object_name);

    void mouseHandler(int event, int x, int y, int left_or_right);

  private:

    bool running;
    bool paused;

    MainController();
};

#endif

