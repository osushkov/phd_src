/*
 * BoxPhysicsObject.cpp
 *
 *  Created on: 04/03/2010
 *      Author: osushkov
 */

#include "BoxPhysicsObject.h"
#include "../Util/Common.h"

BoxPhysicsObject::BoxPhysicsObject(float mass, float width, float height, float depth,
                                   Transform start_pose) :
    width(width), height(height), depth(depth) {



    id = PhysicsObject::getNewObjectId();
    type = POT_BOX;

    transform.shift = Vector3D(0.0f, 0.0f, 0.0f);
    transform.quaternion = btQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
    transform.mat.identity();

    obj_mass = mass;
    transform = start_pose;
}

BoxPhysicsObject::~BoxPhysicsObject(){

}

void BoxPhysicsObject::update(void){
    btTransform trans;
    body->getMotionState()->getWorldTransform(trans);

    transform.shift.x = trans.getOrigin().getX();
    transform.shift.y = trans.getOrigin().getY();
    transform.shift.z = trans.getOrigin().getZ();

    transform.quaternions.clear();
    transform.quaternions.push_back(-trans.getRotation().getW());
    transform.quaternions.push_back(trans.getRotation().getX());
    transform.quaternions.push_back(trans.getRotation().getY());
    transform.quaternions.push_back(trans.getRotation().getZ());

    /*for(unsigned i = 0; i < transform.quaternions.size(); i++){
        std::cout << transform.quaternions[i] << " ";
    }
    std::cout << std::endl;*/

    transform.mat = Common::quaternionsToMatrix(transform.quaternions);
}

void BoxPhysicsObject::addToWorld(btDiscreteDynamicsWorld *world){
    collision_shape = new btBoxShape(btVector3(width/2.0f, height/2.0f, depth/2.0f));
    //collision_shape->setMargin(btScalar(0.001f));

    /// Create Dynamic Objects
    btTransform start_transform;
    start_transform.setIdentity();

    btScalar mass(obj_mass);
    btVector3 local_inertia(0.0f, 0.0f, 0.0f);

    collision_shape->calculateLocalInertia(mass, local_inertia);
    start_transform.setOrigin(btVector3(transform.shift.x, transform.shift.y, transform.shift.z));
    start_transform.setRotation(btQuaternion(transform.quaternions[0], transform.quaternions[1],
                                             transform.quaternions[2], transform.quaternions[3]));

    btDefaultMotionState* my_motion_state = new btDefaultMotionState(start_transform);
    btRigidBody::btRigidBodyConstructionInfo rb_info(mass, my_motion_state, collision_shape, local_inertia);
    body = new btRigidBody(rb_info);
    body->setFriction(0.0f);
    world->addRigidBody(body);
}

void BoxPhysicsObject::removeFromWorld(void){

}
