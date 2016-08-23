
#include "Camera.h"
#include "RemoteCamera.h"
#include "PlaybackCamera.h"
#include "../Settings.h"


Camera* Camera::getLiveCamera(void){
    return new RemoteCamera("humanoid-rightbrain.ai.cse.unsw.edu.au", 9999);
}

Camera* Camera::getPlaybackCamera(std::string path){
    return new PlaybackCamera(path);
}
