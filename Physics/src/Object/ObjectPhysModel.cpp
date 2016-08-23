
#include "ObjectPhysModel.h"
#include "../Visualisation/MeshRenderObject.h"
#include "../Visualisation/SceneRenderer.h"
#include "../Util/Geometry.h"

#include "../Physics/PhysicsWorld.h"
#include "../Physics/PhysicsObject.h"
#include "../Physics/ConvexHullPhysicsObject.h"
#include <cassert>


#define _USE_MATH_DEFINES
#include <math.h>


ObjectPhysModel::ObjectPhysModel(Vector3D box_size, float box_mass, 
                                 std::vector<WheeledBoxPhysicsObject::WheelParams> wheel_params,
                                 std::vector<WheeledBoxPhysicsObject::StubParams> stub_params) : 
    box_size(box_size), box_mass(box_mass), wheel_params(wheel_params), stub_params(stub_params), id(generateId()){

}

ObjectPhysModel::~ObjectPhysModel() {

}

WheeledBoxPhysicsObject* ObjectPhysModel::getPhysicsObject(void){
    return new WheeledBoxPhysicsObject(box_mass, box_size, wheel_params, stub_params, identityTransform());
}

WheeledBoxRenderObject* ObjectPhysModel::getRenderObject(void){
    std::vector<Vector3D> stub_sizes;
    for(unsigned i = 0; i < stub_params.size(); i++){
        stub_sizes.push_back(stub_params[i].size);
    }

    std::vector<std::pair<float,float> > wheel_sizes;
    for(unsigned i = 0; i < wheel_params.size(); i++){
        wheel_sizes.push_back(std::pair<float,float>(wheel_params[i].width, wheel_params[i].radius));
    }

    return new WheeledBoxRenderObject(box_size, stub_sizes, wheel_sizes);
}

unsigned ObjectPhysModel::getId(void) const {
	return id;
}

unsigned ObjectPhysModel::generateId(void){
	static unsigned cur_id;
	return cur_id++;
}
