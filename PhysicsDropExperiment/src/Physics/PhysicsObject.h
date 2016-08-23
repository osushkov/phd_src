/*
 * PhysicsObject.h
 *
 *  Created on: 03/03/2010
 *      Author: osushkov
 */

#ifndef PHYSICSOBJECT_H_
#define PHYSICSOBJECT_H_

#include "../Util/Transform.h"
#include <btBulletDynamicsCommon.h>

#ifndef dTRIMESH_ENABLED
#define dTRIMESH_ENABLED
#endif

enum PhysicsObjectType {
    POT_SPHERE,
    POT_BOX,
    POT_CYLINDER,
    POT_MESH,
    POT_CONVEX_HULL,

    NUM_TYPES
};

class PhysicsObject {
  public:
    static unsigned getNewObjectId(void);
    PhysicsObject(){}
    virtual ~PhysicsObject(){}

    unsigned getId(void) const;
    PhysicsObjectType getType(void) const;
    Transform getTransform(void) const;

    void setVelocity(Vector3D vel);
    void setPose(Transform new_pose);

    void setFriction(float var);
    void setAngularDamping(float var);
    void setLinearDamping(float var);
    void setRestitution(float var);

    float getFriction(void) const;
    float getAngularDamping(void) const;
    float getLinearDamping(void) const;
    float getRestitution(void) const;

    void activate(void);
    bool isActive(void);

    btCollisionShape* getCollisionShape(void);
    btRigidBody* getRigidBody(void);

    virtual void update(void){};
    virtual void addToWorld(btDiscreteDynamicsWorld *world) = 0;
    virtual void removeFromWorld(btDiscreteDynamicsWorld *world) = 0;

    virtual float getLowestSurfacePoint(Quaternion q) = 0;
    virtual void applyImpulse(Vector3D impulse, Vector3D pos);

  protected:
    unsigned id;
    PhysicsObjectType type;
    Transform transform; // the current pose of the object in the simulated world.

    float obj_friction;
    float obj_mass;
    Vector3D obj_cog;

    float obj_angular_damping;
    float obj_linear_damping;
    float obj_restitution;

    btCollisionShape* collision_shape;
    btDefaultMotionState* my_motion_state;
    btRigidBody* body;

};

#endif /* PHYSICSOBJECT_H_ */
