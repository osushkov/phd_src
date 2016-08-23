/*
 * BoxPhysicsObject.h
 *
 *  Created on: 04/03/2010
 *      Author: osushkov
 */

#ifndef BOXPHYSICSOBJECT_H_
#define BOXPHYSICSOBJECT_H_

#include "PhysicsObject.h"
#include "../Util/Vector3D.h"


class BoxPhysicsObject : public PhysicsObject {
  public:
    BoxPhysicsObject(float mass, float width, float height, float depth,
                     Transform start_pose);
    ~BoxPhysicsObject();

    void update(void);
    void addToWorld(btDiscreteDynamicsWorld *world);
    void removeFromWorld(void);

  private:
    float width, height, depth;

};

#endif /* BOXPHYSICSOBJECT_H_ */
