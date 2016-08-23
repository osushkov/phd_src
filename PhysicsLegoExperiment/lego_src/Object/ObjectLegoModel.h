
#ifndef _ObjectLegoModel_H_
#define _ObjectLegoModel_H_

#include "../Util/Vector3D.h"

class ObjectLegoModel {
  public:
    ObjectLegoModel();
    ObjectLegoModel(int left_sensor_turn_direction, int left_sensor_run_direction,
                    int right_sensor_turn_direction, int right_sensor_run_direction,
                    std::pair<Vector3D,Vector3D> left_sensor,
                    std::pair<Vector3D,Vector3D> right_sensor);

    ~ObjectLegoModel();

    int getLeftSensorTurnDirection(void) const;
    int getLeftSensorRunDirection(void) const;

    int getRightSensorTurnDirection(void) const;
    int getRightSensorRunDirection(void) const;

    Vector3D getLeftSensorPos(void) const;
    Vector3D getRightSensorPos(void) const;

    Vector3D getLeftSensorViewDir(void) const;
    Vector3D getRightSensorViewDir(void) const;

    float lightAmountLeftSensor(Vector3D pos, float radius);
    float lightAmountRightSensor(Vector3D pos, float radius);

    unsigned getId(void) const;

  private:

    Vector3D left_sensor_pos;
    Vector3D right_sensor_pos;

    Vector3D left_sensor_view_dir;
    Vector3D right_sensor_view_dir;

    float sensor_view_angle;

    int left_sensor_turn_direction;
    int right_sensor_turn_direction;

    int left_sensor_run_direction;
    int right_sensor_run_direction;

    unsigned id;
    static unsigned generateId(void);

    Vector3D planeIntersectPoint(Vector3D src, Vector3D dir, float plane_z);
};


#endif
