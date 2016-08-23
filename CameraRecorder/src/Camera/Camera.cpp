
#include "Camera.h"
#include "RemoteCamera.h"
#include "PlaybackCamera.h"
//#include "../Settings.h"


Camera* Camera::getLiveCamera(void){
    std::string camera_address("humanoid-rightbrain.ai.cse.unsw.edu.au");
    int camera_port = 9999;
    return new RemoteCamera(camera_address, camera_port);
    /*
    return new RemoteCamera(Settings::instance().getStringValue("general", "camera_address"),
                            Settings::instance().getIntValue("general", "camera_port"));
    */
}

Camera* Camera::getPlaybackCamera(std::string path){
    return new PlaybackCamera(path);
}
