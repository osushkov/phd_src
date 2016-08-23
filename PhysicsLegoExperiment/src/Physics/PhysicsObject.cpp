/*
 * PhysicsObject.cpp
 *
 *  Created on: 03/03/2010
 *      Author: osushkov
 */

#include "PhysicsObject.h"
#include "../Util/Common.h"
#include "../Util/Semaphore.h"

static Util::Semaphore lock(1);
unsigned PhysicsObject::getNewObjectId(void){
    static unsigned cur_id;
    unsigned result;

    lock.wait();
    result = cur_id++;
    lock.signal();
    
    return result;
}

unsigned PhysicsObject::getId(void) const {
    return id;
}

PhysicsObjectType PhysicsObject::getType(void) const {
    return type;
}

Transform PhysicsObject::getTransform(void) const {
    return transform;
}

void PhysicsObject::setVelocity(Vector3D vel){
    body->setLinearVelocity(btVector3(vel.x, vel.y, vel.z));
    body->setGravity(btVector3(0.0f, 0.0f, 0.0f));
    body->setAngularVelocity(btVector3(0.0f, 0.0f, 0.0f));
}

void PhysicsObject::setPose(Transform new_pose){
    transform = new_pose;

    btTransform new_world_transform;
    new_world_transform.setIdentity();
    new_world_transform.setOrigin(btVector3(new_pose.shift.x, new_pose.shift.y, new_pose.shift.z));
    new_world_transform.setRotation(btQuaternion(new_pose.quaternion.x, new_pose.quaternion.y,
                                                 new_pose.quaternion.z, new_pose.quaternion.w));

    my_motion_state->setWorldTransform(new_world_transform);
    //btDefaultMotionState* new_motion_state = new btDefaultMotionState(new_world_transform);
    body->setMotionState(my_motion_state); // TODO: does this even need to be here?

    //body->setWorldTransform(new_world_transform);
}

void PhysicsObject::setFriction(float var){
	obj_friction = var;
	body->setFriction(obj_friction);
}

void PhysicsObject::setAngularDamping(float var){
	obj_angular_damping = var;
	body->setDamping(obj_linear_damping, obj_angular_damping);
}

void PhysicsObject::setLinearDamping(float var){
	obj_linear_damping = var;
	body->setDamping(obj_linear_damping, obj_angular_damping);
}

void PhysicsObject::setRestitution(float var){
	obj_restitution = var;
	body->setRestitution(obj_restitution);
}

float PhysicsObject::getFriction(void) const {
	return obj_friction;
}

float PhysicsObject::getAngularDamping(void) const {
	return obj_angular_damping;
}

float PhysicsObject::getLinearDamping(void) const {
	return obj_linear_damping;
}

float PhysicsObject::getRestitution(void) const {
	return obj_restitution;
}

void PhysicsObject::activate(void){
    body->setActivationState(DISABLE_DEACTIVATION);
    body->activate(true);
}

bool PhysicsObject::isActive(void){
    return body->isActive();
}

btCollisionShape* PhysicsObject::getCollisionShape(void){
    return collision_shape;
}

btRigidBody* PhysicsObject::getRigidBody(void){
    return body;
}

void PhysicsObject::applyImpulse(Vector3D impulse, Vector3D pos){
    btVector3 bt_impulse(impulse.x, impulse.y, impulse.z);
    btVector3 bt_pos(pos.x, pos.y, pos.z);
    body->applyImpulse(bt_impulse, bt_pos);
}