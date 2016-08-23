/*
 * ConvexHullPhysicsObject.h
 *
 *  Created on: 15/03/2010
 *      Author: osushkov
 */

#ifndef CONVEXHULLPHYSICSOBJECT_H_
#define CONVEXHULLPHYSICSOBJECT_H_

#include "PhysicsObject.h"
#include "../Util/Vector3D.h"

class ConvexHullPhysicsObject : public PhysicsObject {
  public:
    ConvexHullPhysicsObject(std::vector<Vector3D> points, float mass, 
                            Transform start_pose);
    ~ConvexHullPhysicsObject();

    void update(void);
    void addToWorld(btDiscreteDynamicsWorld *world);
    void removeFromWorld(btDiscreteDynamicsWorld *world);
    float getLowestSurfacePoint(Quaternion q);

  private:
    std::vector<Vector3D> my_points;
};

#endif /* CONVEXHULLPHYSICSOBJECT_H_ */
