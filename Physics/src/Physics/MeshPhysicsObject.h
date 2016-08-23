/*
 * MeshPhysicsObject.h
 *
 *  Created on: 05/03/2010
 *      Author: osushkov
 */

#ifndef MESHPHYSICSOBJECT_H_
#define MESHPHYSICSOBJECT_H_

#include "PhysicsObject.h"
#include "../Util/Vector3D.h"
#include "../Mesh.h"


class MeshPhysicsObject : public PhysicsObject {
  public:
    MeshPhysicsObject(float mass, Mesh mesh, Transform start_pose);
    ~MeshPhysicsObject();

    void update(void);
    void addToWorld(btDiscreteDynamicsWorld *world);
    void removeFromWorld(void);
    float getLowestSurfacePoint(Quaternion q);

  private:

    Mesh mesh;
};


#endif /* MESHPHYSICSOBJECT_H_ */
