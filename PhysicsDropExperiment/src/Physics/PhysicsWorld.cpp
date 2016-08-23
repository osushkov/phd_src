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

    // Add the ground plane/box.
    btCollisionShape* ground_shape = new btBoxShape(btVector3(btScalar(50.0f),btScalar(50.0f),btScalar(50.0f)));

    btTransform ground_transform;
    ground_transform.setIdentity();
    ground_transform.setOrigin(btVector3(0.0f, 0.0f, -50.0f));

    btScalar mass(0.0f);
    btVector3 local_inertia(0.0f, 0.0f, 0.0f);

    btDefaultMotionState* my_motion_state = new btDefaultMotionState(ground_transform);
    btRigidBody::btRigidBodyConstructionInfo rb_info(mass, my_motion_state, ground_shape, local_inertia);
    btRigidBody* body = new btRigidBody(rb_info);
    body->setFriction(100.0f);

    dynamics_world->addRigidBody(body);

    is_initialised = true;
    lock.signal();
};


void PhysicsWorld::step(void){
    assert(is_initialised);
    assert(lock.getCount() == 1);

    lock.wait();
    //std::cout << "World objects: " << dynamics_world->getNumCollisionObjects() << " " << all_objects.size() << std::endl;

    dynamics_world->stepSimulation(1.0f/30.0f);
    //std::cout << dynamics_world->getNumCollisionObjects() << std::endl;
    //btVector3 g = dynamics_world->getGravity();
    //std::cout << g.m_floats[0] << " " << g.m_floats[1] << " " << g.m_floats[2] << std::endl;
    //btCollisionObjectArray a = dynamics_world->getCollisionObjectArray();
    //std::cout << a.size() << "!" << std::endl;

    std::map<unsigned, PhysicsObject*>::iterator it;
    //std::cout << all_objects.size() << "!" << std::endl;

    for(it = all_objects.begin(); it != all_objects.end(); ++it){
        it->second->update();
    }

    lock.signal();
}

unsigned PhysicsWorld::addObject(PhysicsObject *new_object){
    assert(is_initialised);
    assert(new_object != NULL);

    assert(lock.getCount() == 1);
    lock.wait();

    new_object->addToWorld(dynamics_world);
    all_objects.insert(std::pair<unsigned,PhysicsObject*>(new_object->getId(), new_object));
    unsigned result = new_object->getId();

    lock.signal();
    return result;
}

void PhysicsWorld::addConstraint(btGeneric6DofConstraint *constraint){
    dynamics_world->addConstraint(constraint);
}

bool PhysicsWorld::removeObject(unsigned id){
    assert(is_initialised);
    assert(lock.getCount() == 1);
    lock.wait();

    std::map<unsigned, PhysicsObject*>::iterator it = all_objects.find(id);
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
    std::map<unsigned, PhysicsObject*>::iterator it;
    for(it = all_objects.begin(); it != all_objects.end(); ++it){
        it->second->removeFromWorld(dynamics_world);
        //delete it->second;
    }
    all_objects.clear();
    lock.signal();
}

unsigned PhysicsWorld::getId(void) const {
    return id;
}

PhysicsObject* PhysicsWorld::getObject(unsigned id){
    assert(is_initialised);
    std::map<unsigned, PhysicsObject*>::iterator it = all_objects.find(id);
    if(it == all_objects.end()){
        std::cout << "object not found: " << id << std::endl;
        return NULL;
    }
    else{
        return it->second;
    }
}

