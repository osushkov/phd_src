/*
 * PhysicsWorld.h
 *
 *  Created on: 02/03/2010
 *      Author: osushkov
 */

#ifndef PHYSICSWORLD_H_
#define PHYSICSWORLD_H_

#include "../Util/Transform.h"
#include "../Util/Semaphore.h"
#include "WheeledBoxPhysicsObject.h"
#include <btBulletDynamicsCommon.h>
#include <map>

#ifndef dTRIMESH_ENABLED
#define dTRIMESH_ENABLED
#endif

class PhysicsWorld {
  public:
    PhysicsWorld(unsigned id);
    ~PhysicsWorld();

    void initialise(void);
    void step(void);

    unsigned addObject(WheeledBoxPhysicsObject *new_object);
    void addConstraint(btGeneric6DofConstraint *constraint);
    void addRigidBody(btRigidBody* rigid_body);

    bool removeObject(unsigned id);
    WheeledBoxPhysicsObject* getObject(unsigned id);

    void clear(void);
    unsigned getId(void) const;

  private:
    const unsigned id;
    bool is_initialised;
    Util::Semaphore lock;

    btDefaultCollisionConfiguration *collision_conf;
    btCollisionDispatcher *dispatcher;
    btBroadphaseInterface *overlapping_pair_cache;
    btSequentialImpulseConstraintSolver *solver;
    btDiscreteDynamicsWorld* dynamics_world;

    std::map<unsigned, WheeledBoxPhysicsObject*> all_objects;
};


#endif /* PHYSICSWORLD_H_ */
