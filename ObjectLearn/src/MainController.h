
#ifndef _MainController_H_
#define _MainController_H_

#include "Camera/Camera.h"
#include "Features/SIFTGenerator.h"
#include "Features/StereoFeatureCorrelation.h"

#include <string>

class MainController {
  public:
    static MainController& instance(void);

    void initialise(void);
    void reload(void);

    void learnObject(std::string object_name);
    void examineObject(std::string object_name);

    void correlateObjects(std::string object_name);
    void generateSIFT(std::string path);
    void test(void);
    void generateMovement(std::string record_path);
    void rotate(std::string record_path);
    void sceneMatchObject(std::string object_name);

    void stitchTogether(std::string object0, std::string object1);
    void silhouetteObject(std::string object_name);

    void mouseHandler(int event, int x, int y, int left_or_right);

  private:

    bool running;
    bool paused;

    unsigned cur_frame;
    Camera *cam;
    unsigned cam_width;
    unsigned cam_height;

    std::vector<unsigned char> left_cam_buffer;
    std::vector<unsigned char> right_cam_buffer;
    IplImage* img_left;
    IplImage* img_right;

    SIFTGenerator *sift_generator;
    std::vector<feature> left_sift_features, right_sift_features;
    std::vector<StereoFeature> stereo_features;


    std::vector<StereoFeature> frame1_features, frame2_features;

    MainController();

    void loadCamera(bool live_camera,std::string video_path="");
    void loadSIFTGenerator(bool live_camera, std::string video_path="");

    bool getNewFrame(bool generate_sift_features);
    char showCameraViewAndContinue(void);

    void createViewWindows(void);
    void closeWindow(std::string name);


};

#endif

