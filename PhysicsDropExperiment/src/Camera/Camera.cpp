
#include "Camera.h"
#include "RemoteCamera.h"
#include "PlaybackCamera.h"
#include "../Settings.h"


Camera* Camera::getLiveCamera(void){
    return new RemoteCamera(Settings::instance().getStringValue("general", "camera_address"),
                            Settings::instance().getIntValue("general", "camera_port"));
}

Camera* Camera::getPlaybackCamera(std::string path){
    return new PlaybackCamera(path);
}
