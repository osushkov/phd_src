
#include "CompoundPhysicsObject.h"
#include "PhysicsObject.h"
#include <cassert>

CompoundPhysicsObject::CompoundPhysicsObject(std::vector<PhysicsObject*> sub_objects, 
                                             std::vector<Vector3D> rel_positions){

    this->sub_objects = sub_objects;
    this->rel_positions = rel_positions;
    id = 1234;
}

CompoundPhysicsObject::~CompoundPhysicsObject(){
    for(unsigned i = 0; i < sub_objects.size(); i++){
        delete sub_objects[i];
    }

    for(unsigned i = 0; i < constraints.size(); i++){
        delete constraints[i];
    }
}

void CompoundPhysicsObject::addToWorld(PhysicsWorld *world){
    assert(world != NULL);
    
    for(unsigned i = 0; i < sub_objects.size(); i++){
        world->addObject(sub_objects[i]);
    }

    for(unsigned i = 1; i < sub_objects.size(); i++){
        btTransform transform0;
        transform0.setIdentity();
        transform0.setOrigin(btVector3(rel_positions[i].x, rel_positions[i].y, rel_positions[i].z));

        btTransform transform1;
        transform1.setIdentity();
        transform1.setOrigin(btVector3(0.0f, 0.0f, 0.0f));

        btGeneric6DofConstraint *new_constraint = 
            new btGeneric6DofConstraint(*sub_objects[0]->getRigidBody(), *sub_objects[i]->getRigidBody(), transform0, transform1, true);

        new_constraint->setLinearLowerLimit(btVector3(0.0f, 0.0f, 0.0f));
        new_constraint->setLinearUpperLimit(btVector3(0.0f, 0.0f, 0.0f));

        new_constraint->setAngularLowerLimit(btVector3(0.0f, 0.0f, 0.0f));
        new_constraint->setAngularUpperLimit(btVector3(0.0f, 0.0f, 0.0f));

        constraints.push_back(new_constraint);
    }

    for(unsigned i = 0; i < constraints.size(); i++){
        world->addConstraint(constraints[i]);
    }
}

unsigned CompoundPhysicsObject::getId(void){
    return id;
}