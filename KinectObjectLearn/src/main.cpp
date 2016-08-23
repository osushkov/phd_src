

#include "MainController.h"
#include "Arm/Arm.h"
#include "Settings.h"
#include "Console.h"
#include "Control.h"
#include "Util/Octree.hpp"

#include "Util/Common.h"
#include "Util/Geometry.h"
#include "Util/Vector3D.h"
#include "Util/Vector2D.h"


Vector2D projectPointToCamera2(Vector3D cam_pos, Vector3D point_pos){
    float plane_intersect_t = 0.0f;
    Vector3D cam_view_dir(0.0f, -0.707f, -0.707f);
    Geometry::linePlaneIntersect(point_pos, cam_pos, cam_view_dir, 1.0f, plane_intersect_t);
    Vector3D result_3d = point_pos + plane_intersect_t*(cam_pos-point_pos);
    return Vector2D(result_3d.x-cam_pos.x, result_3d.z-cam_pos.z);
}

Vector2D pointWorldPosToPixelPos2(Vector2D point_pos, float scale_factor){
    float theta = Settings::instance().getFloatValue("camera", "xfov")/2.0f;
    float phi   = Settings::instance().getFloatValue("camera", "yfov")/2.0f;

    float width  = 640.0f;
    float height = 480.0f;

    float lx = -tan(theta*M_PI/180.0f);
    float rx = tan(theta*M_PI/180.0f);

    float ty = tan(phi*M_PI/180.0f);
    float by = -tan(phi*M_PI/180.0f);

    float xr = (point_pos.x - lx)/(rx - lx);
    float yr = (point_pos.y - ty)/(by - ty);

    return Vector2D(width - xr*width, height - yr*height);
}

int main(int argc, char **argv){
    Settings::instance().load("data/settings.cfg");

#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(1,1), &wsaData) != 0) {
        std::cerr << "WSAStartup failed." << std::endl;
        return false;
    }
    else {
        std::cout << "WSAStartup succeeded" << std::endl;
    }
#endif

    //Control::moveArmOutOfTheWay();
    //return 0;
/*
    Vector3D point(1.0f, 10.0f, -1.0f);
    //point.scale(100.0f);
    point.z = -1.0f;
    Vector2D pixel_pos = pointWorldPosToPixelPos2(projectPointToCamera2(Vector3D(0.0f, 0.0f, 0.0f), point), 1.0f);
    pixel_pos.print();
    return 0;
*/


    Arm::getArm(); // basically connect to the arm
    MainController::instance().initialise();

    // Start up the console. Keep prompting the user for an input command.
    // When the user performs the quet command, prompt() returns false and we
    // exit.
    while(Console::instance().prompt());

    // TODO: maybe print out some statistics/summary here of the session
    return 0;
}
