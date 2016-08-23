
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




ObjectPhysModel::ObjectPhysModel() : id(generateId()){}

ObjectPhysModel::ObjectPhysModel(SuperQuadric shape, float mass, Vector3D cog, float c_friction) :
        shape(shape), mass(mass), cog(cog), c_friction(c_friction), id(generateId()) {}

ObjectPhysModel::~ObjectPhysModel() {

}

PhysicsObject* ObjectPhysModel::getPhysicsObject(void){
    
    std::vector<Vector3D> sq_points = shape.getSurfacePoints((float)M_PI/10.0f);
    for(unsigned i = 0; i < sq_points.size(); i++){
        sq_points[i] = sq_points[i] - cog;
    }

    Transform start_pose;
    start_pose.shift = Vector3D(0.0f, 0.0f, 0.0f);
    start_pose.quaternion = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
    
    PhysicsObject *result = new ConvexHullPhysicsObject(sq_points, mass, start_pose);
    return result;
}

RenderObject* ObjectPhysModel::getRenderObject(void){
    Mesh sq_mesh = shape.convertToMesh((float)M_PI/10.0f);
    for(unsigned i = 0; i < sq_mesh.vertices.size(); i++){
        sq_mesh.vertices[i] = sq_mesh.vertices[i] - cog;
    }

    return new MeshRenderObject(sq_mesh);
}

unsigned ObjectPhysModel::getId(void) const {
	return id;
}

unsigned ObjectPhysModel::generateId(void){
	static unsigned cur_id;
	return cur_id++;
}
