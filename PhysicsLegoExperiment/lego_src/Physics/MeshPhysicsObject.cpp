/*
 * MeshPhysicsObject.cpp
 *
 *  Created on: 05/03/2010
 *      Author: osushkov
 */

#include "MeshPhysicsObject.h"


MeshPhysicsObject::MeshPhysicsObject(float mass, Mesh mesh, Transform start_pose) :
    mesh(mesh) {

    id = PhysicsObject::getNewObjectId();
    type = POT_MESH;

    obj_mass = mass;
    transform = start_pose;
}

MeshPhysicsObject::~MeshPhysicsObject(){

}

void MeshPhysicsObject::update(void){

}


void MeshPhysicsObject::addToWorld(btDiscreteDynamicsWorld *world){

}

void MeshPhysicsObject::removeFromWorld(void){

}

float MeshPhysicsObject::getLowestSurfacePoint(Quaternion q){
    return 0.0f;
}