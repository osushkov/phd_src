/*
 * ConvexHullPhysicsObject.cpp
 *
 *  Created on: 15/03/2010
 *      Author: osushkov
 */

#include "ConvexHullPhysicsObject.h"
#include "WorldParams.h"
#include "../Util/Common.h"
#include "../Util/Semaphore.h"

ConvexHullPhysicsObject::ConvexHullPhysicsObject(std::vector<Vector3D> points, float mass,
                                                 Transform start_pose) :
    my_points(points) {

    id = PhysicsObject::getNewObjectId();
    type = POT_CONVEX_HULL;

    obj_mass = mass;
    transform = start_pose;

    obj_friction = FRICTION;
    obj_restitution = RESTITUTION;
    obj_linear_damping = LINEAR_DAMPING;
    obj_angular_damping = ANGULAR_DAMPING;
   
    collision_shape = new btConvexHullShape(NULL, 0);
    
    for(unsigned i = 0; i < my_points.size(); i++){
        btVector3 v(btScalar(my_points[i].x), btScalar(my_points[i].y), btScalar(my_points[i].z));
        ((btConvexHullShape*)collision_shape)->addPoint(v);
    }
        
    collision_shape->setMargin(btScalar(0.001f));
    
    /// Create Dynamic Objects
    btTransform start_transform;
    start_transform.setIdentity();

    btScalar pmass(obj_mass);
    btVector3 local_inertia(0.0f, 0.0f, 0.0f);

    collision_shape->calculateLocalInertia(pmass, local_inertia);
    start_transform.setOrigin(btVector3(transform.shift.x, transform.shift.y, transform.shift.z));
    start_transform.setRotation(btQuaternion(transform.quaternion.x, transform.quaternion.y,
                                             transform.quaternion.z, transform.quaternion.w));

    my_motion_state = new btDefaultMotionState(start_transform);
    btRigidBody::btRigidBodyConstructionInfo rb_info(pmass, my_motion_state, collision_shape, local_inertia);
    body = new btRigidBody(rb_info);
    body->setFriction(obj_friction);
    body->setDamping(obj_linear_damping, obj_angular_damping);
    body->setRestitution(obj_restitution);
}

ConvexHullPhysicsObject::~ConvexHullPhysicsObject(){
    delete body;
    delete my_motion_state;
    delete collision_shape;
}

void ConvexHullPhysicsObject::update(void){
    btTransform trans;
    body->getMotionState()->getWorldTransform(trans);

    transform.shift.x = trans.getOrigin().getX();
    transform.shift.y = trans.getOrigin().getY();
    transform.shift.z = trans.getOrigin().getZ();

    btQuaternion tmp = trans.getRotation();
    transform.quaternion = Quaternion(tmp.getX(), tmp.getY(), tmp.getZ(), tmp.getW());
    transform.mat = transform.quaternion.toMatrix();
}

void ConvexHullPhysicsObject::addToWorld(btDiscreteDynamicsWorld *world){
    world->addRigidBody(body);
}

void ConvexHullPhysicsObject::removeFromWorld(btDiscreteDynamicsWorld *world){
    world->removeRigidBody(body);
}

float ConvexHullPhysicsObject::getLowestSurfacePoint(Quaternion q){
    Matrix3 mat = q.toMatrix();

    float result = 0.0f;
    for(unsigned i = 0; i < my_points.size(); i++){
        Vector3D tp = mat*my_points[i];
        if(i == 0 || tp.z < result){
            result = tp.z;
        }
    }
    return result;
}