
#include "ObjectLegoModel.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include "../Util/Geometry.h"

ObjectLegoModel::ObjectLegoModel() {
    ObjectLegoModel(0, 0, 0, 0, 
        std::pair<Vector3D,Vector3D>(Vector3D(), Vector3D()),
        std::pair<Vector3D,Vector3D>(Vector3D(), Vector3D()));
}

ObjectLegoModel::ObjectLegoModel(int left_sensor_turn_direction, int left_sensor_run_direction,
                                 int right_sensor_turn_direction, int right_sensor_run_direction,
                                 std::pair<Vector3D,Vector3D> left_sensor,
                                 std::pair<Vector3D,Vector3D> right_sensor) : 
    left_sensor_turn_direction(left_sensor_turn_direction), right_sensor_turn_direction(right_sensor_turn_direction),
    left_sensor_run_direction(left_sensor_run_direction), right_sensor_run_direction(right_sensor_run_direction), 
    id(generateId()) {

    left_sensor_pos = left_sensor.first;
    right_sensor_pos = right_sensor.first;

    //left_sensor_pos = Vector3D(5.5f, 5.5f, 0.0f);
    //right_sensor_pos = Vector3D(5.5f, -5.5f, 0.0f);

    left_sensor_view_dir = left_sensor.second;
    right_sensor_view_dir = right_sensor.second;

    left_sensor_view_dir.normalise();
    right_sensor_view_dir.normalise();

    //left_sensor_view_dir = Vector3D(0.707f, 0.0f, -0.707);
    //right_sensor_view_dir = Vector3D(0.707f, 0.0f, -0.707);

    sensor_view_angle = 45.0f * M_PI/180.0f;
}

ObjectLegoModel::~ObjectLegoModel(){

}

int ObjectLegoModel::getLeftSensorTurnDirection(void) const {
    return left_sensor_turn_direction;
}

int ObjectLegoModel::getLeftSensorRunDirection(void) const {
    return left_sensor_run_direction;
}

int ObjectLegoModel::getRightSensorTurnDirection(void) const {
    return right_sensor_turn_direction;
}

int ObjectLegoModel::getRightSensorRunDirection(void) const {
    return right_sensor_run_direction;
}

Vector3D ObjectLegoModel::getLeftSensorPos(void) const {
    return left_sensor_pos;
}

Vector3D ObjectLegoModel::getRightSensorPos(void) const {
    return right_sensor_pos;
}

Vector3D ObjectLegoModel::getLeftSensorViewDir(void) const {
    return left_sensor_view_dir;
}

Vector3D ObjectLegoModel::getRightSensorViewDir(void) const {
    return right_sensor_view_dir;
}

float ObjectLegoModel::lightAmountLeftSensor(Vector3D pos, float radius){
    std::pair<Vector3D,Vector3D> basis_vecs = Geometry::perpedicularBasis(left_sensor_view_dir);

    unsigned sum = 0;
    const unsigned iters = 50;
    for(unsigned i = 0; i < iters; i++){
        Vector2D basis_scal;
        while(true){
            basis_scal.x = (2.0f*(float)rand()/(float)RAND_MAX) - 1.0f;
            basis_scal.y = (2.0f*(float)rand()/(float)RAND_MAX) - 1.0f;

            if(basis_scal.length() < 1.0f){
                break;
            }
        }

        Vector3D offset_vec = basis_scal.x * basis_vecs.first + basis_scal.y * basis_vecs.second;
        offset_vec.normalise();

        float cone_r = Common::uniformNoise(0.0f, tanf(sensor_view_angle));

        Vector3D v = left_sensor_view_dir + cone_r * offset_vec;
        v.normalise();
/*
        v.print();
        left_sensor_view_dir.print();
        std::cout << std::endl;
        getchar();
*/
        Vector3D centre = planeIntersectPoint(left_sensor_pos, v, pos.z);
        if((centre - pos).length() < radius){
            sum++;
        }
    }

    return (float)sum/(float)iters;
}

float ObjectLegoModel::lightAmountRightSensor(Vector3D pos, float radius){
    std::pair<Vector3D,Vector3D> basis_vecs = Geometry::perpedicularBasis(right_sensor_view_dir);

    unsigned sum = 0;
    const unsigned iters = 50;
    for(unsigned i = 0; i < iters; i++){
        Vector2D basis_scal;
        while(true){
            basis_scal.x = (2.0f*(float)rand()/(float)RAND_MAX) - 1.0f;
            basis_scal.y = (2.0f*(float)rand()/(float)RAND_MAX) - 1.0f;

            if(basis_scal.length() < 1.0f){
                break;
            }
        }

        Vector3D offset_vec = basis_scal.x * basis_vecs.first + basis_scal.y * basis_vecs.second;
        offset_vec.normalise();

        float cone_r = Common::uniformNoise(0.0f, tanf(sensor_view_angle));

        Vector3D v = right_sensor_view_dir + cone_r * offset_vec;
        v.normalise();
/*
        v.print();
        right_sensor_view_dir.print();
        std::cout << std::endl;
        getchar();
*/
        Vector3D centre = planeIntersectPoint(right_sensor_pos, v, pos.z);
        if((centre - pos).length() < radius){
            sum++;
        }
    }

    return (float)sum/(float)iters;
}

unsigned ObjectLegoModel::getId(void) const {
    return id;
}

unsigned ObjectLegoModel::generateId(void){
    static unsigned cur_id;
    return cur_id++;
}

Vector3D ObjectLegoModel::planeIntersectPoint(Vector3D src, Vector3D dir, float plane_z){
    float d = (plane_z - src.z)/dir.z;
    return src + d*dir;
}
