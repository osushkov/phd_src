/*
 * PhysicsWorld.cpp
 *
 *  Created on: 02/03/2010
 *      Author: osushkov
 */

#include "PhysicsWorld.h"
#include "../Util/Common.h"
#include "../Util/Timer.h"
#include <btBulletDynamicsCommon.h>
#include <cassert>


PhysicsWorld::PhysicsWorld(unsigned id) : id(id), is_initialised(false), lock(1) {

}

PhysicsWorld::~PhysicsWorld(){
    assert(false);
}

void PhysicsWorld::initialise(void){
    if(is_initialised){ return; }
    lock.wait();

    collision_conf = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collision_conf);
    overlapping_pair_cache = new btDbvtBroadphase();
    solver = new btSequentialImpulseConstraintSolver;

    dynamics_world = new btDiscreteDynamicsWorld(dispatcher,overlapping_pair_cache,solver,collision_conf);
    dynamics_world->setGravity(btVector3(0.0f, 0.0f, -9.8f));

    // Add the ground plane
    
    btStaticPlaneShape *ground_plane = new btStaticPlaneShape(btVector3(0.0f, 0.0f, 1.0f), btScalar(GROUND_HEIGHT)); 
    ground_plane->setMargin(btScalar(0.001f));

    btDefaultMotionState* ground_motion_state = 
        new btDefaultMotionState(btTransform(btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),btVector3(0.0f, 0.0f, 0.0f)));

    btRigidBody::btRigidBodyConstructionInfo ground_rigid_body_CI(0, ground_motion_state, ground_plane, btVector3(0.0f, 0.0f, 0.0f));
    btRigidBody* ground_rigid_body = new btRigidBody(ground_rigid_body_CI);
    ground_rigid_body->setFriction(0.7f);

    dynamics_world->addRigidBody(ground_rigid_body);

    is_initialised = true;
    lock.signal();
};


void PhysicsWorld::step(void){
    assert(is_initialised);

    lock.wait();

    dynamics_world->stepSimulation(1.0f/30.0f, 1, 1.0f/30.0f);

    std::map<unsigned, WheeledBoxPhysicsObject*>::iterator it;
    for(it = all_objects.begin(); it != all_objects.end(); ++it){
        it->second->update();
    }

    lock.signal();
}

unsigned PhysicsWorld::addObject(WheeledBoxPhysicsObject *new_object){
    assert(is_initialised);
    assert(new_object != NULL);

    assert(lock.getCount() == 1);
    lock.wait();

    new_object->addToWorld(dynamics_world);
    all_objects.insert(std::pair<unsigned,WheeledBoxPhysicsObject*>(new_object->getId(), new_object));
    unsigned result = new_object->getId();

    lock.signal();
    return result;
}

void PhysicsWorld::addConstraint(btGeneric6DofConstraint *constraint){
    assert(constraint != NULL);
    dynamics_world->addConstraint(constraint);
}

void PhysicsWorld::addRigidBody(btRigidBody* rigid_body){
    assert(rigid_body != NULL);
    dynamics_world->addRigidBody(rigid_body);
}

bool PhysicsWorld::removeObject(unsigned id){
    assert(is_initialised);
    assert(lock.getCount() == 1);
    lock.wait();

    std::map<unsigned, WheeledBoxPhysicsObject*>::iterator it = all_objects.find(id);
    if(it == all_objects.end()){
        lock.signal();
        return false;
    }
    else{
        it->second->removeFromWorld(dynamics_world);
        all_objects.erase(it);
        lock.signal();
        return true;
    }
}

void PhysicsWorld::clear(void){
    assert(is_initialised);
    assert(lock.getCount() == 1);

    lock.wait();
    std::map<unsigned, WheeledBoxPhysicsObject*>::iterator it;
    for(it = all_objects.begin(); it != all_objects.end(); ++it){
        it->second->removeFromWorld(dynamics_world);
    }
    all_objects.clear();
    lock.signal();
}

unsigned WheeledBoxPhysicsObject::getId(void) const {
    return id;
}

WheeledBoxPhysicsObject* PhysicsWorld::getObject(unsigned id){
    assert(is_initialised);
    std::map<unsigned, WheeledBoxPhysicsObject*>::iterator it = all_objects.find(id);
    if(it == all_objects.end()){
        std::cout << "object not found: " << id << std::endl;
        return NULL;
    }
    else{
        return it->second;
    }
}

