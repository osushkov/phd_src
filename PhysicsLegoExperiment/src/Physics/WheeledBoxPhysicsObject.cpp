
#include "WheeledBoxPhysicsObject.h"
#include "PhysicsObject.h"
#include "WorldParams.h"
#include "../Util/Geometry.h"

//using class WheeledBoxPhysicsObject;

WheeledBoxPhysicsObject::WheeledBoxPhysicsObject(float box_mass, 
                                                 Vector3D box_size, 
                                                 std::vector<WheelParams> wheel_params,
                                                 std::vector<StubParams> stub_params, 
                                                 Transform start_pose) :
    box_mass(box_mass), box_size(box_size) {

    id = PhysicsObject::getNewObjectId();

    float obj_restitution = RESTITUTION;
    float obj_linear_damping = LINEAR_DAMPING;
    float obj_angular_damping = ANGULAR_DAMPING;

    box_shape = new btBoxShape(btVector3(box_size.x, box_size.y, box_size.z)); 
    box_shape->setMargin(btScalar(0.001f));

    btTransform start_transform;
    start_transform.setOrigin(btVector3(start_pose.shift.x, start_pose.shift.y, start_pose.shift.z));
    start_transform.setRotation(btQuaternion(start_pose.quaternion.x, start_pose.quaternion.y,
                                             start_pose.quaternion.z, start_pose.quaternion.w));

    btVector3 box_inertia(0.0f, 0.0f, 0.0f);
    box_shape->calculateLocalInertia(box_mass, box_inertia);

    box_motion_state = new btDefaultMotionState(start_transform);

    btRigidBody::btRigidBodyConstructionInfo box_body_CI(box_mass, box_motion_state, box_shape, box_inertia);
    box_body = new btRigidBody(box_body_CI);
    box_body->setFriction(0.6f);
    //box_body->setDamping(obj_linear_damping, obj_angular_damping);
    box_body->setRestitution(obj_restitution);


    for(unsigned i = 0; i < wheel_params.size(); i++){
        Wheel new_wheel;
        new_wheel.params = wheel_params[i];
        new_wheel.shape = new btCylinderShapeX(btVector3(wheel_params[i].width/2.0f, wheel_params[i].radius, wheel_params[i].radius));
        new_wheel.shape->setMargin(btScalar(0.001f));

        btVector3 wheel_inertia(0.0f, 0.0f, 0.0f);
        new_wheel.shape->calculateLocalInertia(box_mass, wheel_inertia);

        new_wheel.motion_state = new btDefaultMotionState(btTransform(btQuaternion(0.0f, 0.0f, 0.0f, 1.0f), btVector3(0.0f, 0.0f, 0.0f)));

        btRigidBody::btRigidBodyConstructionInfo wheel_body_CI(wheel_params[i].mass, new_wheel.motion_state, 
                                                               new_wheel.shape, wheel_inertia);
        new_wheel.body = new btRigidBody(wheel_body_CI);
        new_wheel.body->setFriction(wheel_params[i].friction);
        new_wheel.body->setDamping(obj_linear_damping, wheel_params[i].damping);
        new_wheel.body->setRestitution(obj_restitution);

        buildWheelConstraints(new_wheel);
        wheels.push_back(new_wheel);
    }

    for(unsigned i = 0; i < stub_params.size(); i++){
        Stub new_stub;
        new_stub.params = stub_params[i];
        new_stub.shape = new btBoxShape(btVector3(stub_params[i].size.x, stub_params[i].size.y, stub_params[i].size.z));
        new_stub.shape->setMargin(btScalar(0.001f));

        btVector3 stub_inertia(0.0f, 0.0f, 0.0f);
        new_stub.shape->calculateLocalInertia(box_mass, stub_inertia);

        new_stub.motion_state = new btDefaultMotionState(btTransform(btQuaternion(0.0f, 0.0f, 0.0f, 1.0f), btVector3(0.0f, 0.0f, 0.0f)));

        btRigidBody::btRigidBodyConstructionInfo stub_body_CI(stub_params[i].mass, new_stub.motion_state, 
                                                              new_stub.shape, stub_inertia);
        new_stub.body = new btRigidBody(stub_body_CI);
        new_stub.body->setFriction(stub_params[i].friction);
        new_stub.body->setDamping(obj_linear_damping, obj_angular_damping);
        new_stub.body->setRestitution(obj_restitution);

        buildStubConstraint(new_stub);
        stubs.push_back(new_stub);
    }
}

WheeledBoxPhysicsObject::~WheeledBoxPhysicsObject(){
    delete box_body;
    delete box_motion_state;
    delete box_shape;

    for(unsigned i = 0; i < stubs.size(); i++){
        delete stubs[i].joint;
        delete stubs[i].body;
        delete stubs[i].shape;
        delete stubs[i].motion_state;
    }

    for(unsigned i = 0; i < wheels.size(); i++){
        delete wheels[i].joint;
        delete wheels[i].body;
        delete wheels[i].shape;
        delete wheels[i].motion_state;
    }
}

void WheeledBoxPhysicsObject::update(void){
    updateBox();
    updateStubs();
    updateWheels();
}

void WheeledBoxPhysicsObject::addToWorld(btDiscreteDynamicsWorld *world){
    world->addRigidBody(box_body);

    for(unsigned i = 0; i < stubs.size(); i++){
        world->addRigidBody(stubs[i].body);
        world->addConstraint(stubs[i].joint);
    }

    for(unsigned i = 0; i < wheels.size(); i++){
        world->addRigidBody(wheels[i].body);
        world->addConstraint(wheels[i].joint);
    }
}

void WheeledBoxPhysicsObject::removeFromWorld(btDiscreteDynamicsWorld *world){
    for(unsigned i = 0; i < stubs.size(); i++){
        world->removeConstraint(stubs[i].joint);
        world->removeRigidBody(stubs[i].body);
    }

    for(unsigned i = 0; i < wheels.size(); i++){
        world->removeConstraint(wheels[i].joint);
        world->removeRigidBody(wheels[i].body);
    }

    world->removeRigidBody(box_body);
}

void WheeledBoxPhysicsObject::moveTo(Transform box_transform){
    this->box_transform = box_transform;
    setTransform(this->box_transform, box_motion_state, box_body);

    for(unsigned i = 0; i < stubs.size(); i++){
        stubs[i].transform.shift = box_transform.shift + box_transform.mat * stubs[i].params.position;
        stubs[i].transform.mat = box_transform.mat;
        stubs[i].transform.quaternion = box_transform.quaternion;

        setTransform(stubs[i].transform, stubs[i].motion_state, stubs[i].body);
    }

    for(unsigned i = 0; i < wheels.size(); i++){
        wheels[i].transform.shift = box_transform.shift + box_transform.mat * wheels[i].params.position;
        wheels[i].transform.mat = box_transform.mat * Geometry::getMatrixFromTo(Vector3D(1.0f, 0.0f, 0.0f), wheels[i].params.axle);
        wheels[i].transform.quaternion = Quaternion(wheels[i].transform.mat);
        
        setTransform(wheels[i].transform, wheels[i].motion_state, wheels[i].body);
    }
}

float WheeledBoxPhysicsObject::getLowestSurfacePoint(Quaternion q){

}

std::vector<Transform> WheeledBoxPhysicsObject::getStubTransforms(void){
    std::vector<Transform> result;
    for(unsigned i = 0; i < stubs.size(); i++){
        result.push_back(stubs[i].transform);
    }
    return result;
}

std::vector<Transform> WheeledBoxPhysicsObject::getWheelTransforms(void){
    std::vector<Transform> result;
    for(unsigned i = 0; i < wheels.size(); i++){
        result.push_back(wheels[i].transform);
    }
    return result;
}

Transform WheeledBoxPhysicsObject::getBoxTransform(void){
    return box_transform;
}

float WheeledBoxPhysicsObject::getBoxBottomOffset(void){
    float max_stub_wheel = 0.0f;
    for(unsigned i = 0; i < stubs.size(); i++){
        float stub_offset = fabs(stubs[i].params.position.z) + stubs[i].params.size.z;
        if(stub_offset > max_stub_wheel){
            max_stub_wheel = stub_offset;
        }
    }

    for(unsigned i = 0; i < wheels.size(); i++){
        float wheel_offset = fabs(wheels[i].params.position.z) + wheels[i].params.radius;
        if(wheel_offset > max_stub_wheel){
            max_stub_wheel = wheel_offset;
        }
    }
    
    return std::max<float>(max_stub_wheel, box_size.z) + 0.001f;
}

void WheeledBoxPhysicsObject::activate(void){
    box_body->activate();
    for(unsigned i = 0; i < wheels.size(); i++){
        wheels[i].body->activate();
    }

    for(unsigned i = 0; i < stubs.size(); i++){
        stubs[i].body->activate();
    }
}

bool WheeledBoxPhysicsObject::isActive(void){
    return box_body->isActive();
}

void WheeledBoxPhysicsObject::buildStubConstraint(Stub &stub){
    btTransform transform0;
    transform0.setIdentity();
    transform0.setOrigin(btVector3(stub.params.position.x, stub.params.position.y, stub.params.position.z));

    btTransform transform1;
    transform1.setIdentity();
    transform1.setOrigin(btVector3(0.0f, 0.0f, 0.0f));

    stub.joint = new btGeneric6DofConstraint(*box_body, *stub.body, transform0, transform1, true);

    stub.joint->setLinearLowerLimit(btVector3(0.0f, 0.0f, 0.0f));
    stub.joint->setLinearUpperLimit(btVector3(0.0f, 0.0f, 0.0f));

    stub.joint->setAngularLowerLimit(btVector3(0.0f, 0.0f, 0.0f));
    stub.joint->setAngularUpperLimit(btVector3(0.0f, 0.0f, 0.0f));
}

void WheeledBoxPhysicsObject::buildWheelConstraints(Wheel &wheel){
    btVector3 transform_A(wheel.params.position.x, wheel.params.position.y, wheel.params.position.z);
    btVector3 transform_B(0.0f, 0.0f, 0.0f);

    btVector3 hinge_A(wheel.params.axle.x, wheel.params.axle.y, wheel.params.axle.z);
    btVector3 hinge_B(1.0f, 0.0f, 0.0f);

    wheel.joint = new btHingeConstraint(*box_body, *wheel.body, transform_A, transform_B, hinge_A, hinge_B, true);
    //wheel.joint->setLimit(wheel.joint->getLowerLimit(), wheel.joint->getUpperLimit(), 0.9, 0.3, wheel.params.damping);
}

void WheeledBoxPhysicsObject::updateBox(void){
    btTransform trans;
    box_body->getMotionState()->getWorldTransform(trans);

    box_transform.shift.x = trans.getOrigin().getX();
    box_transform.shift.y = trans.getOrigin().getY();
    box_transform.shift.z = trans.getOrigin().getZ();

    btQuaternion tmp = trans.getRotation();
    box_transform.quaternion = Quaternion(tmp.getX(), tmp.getY(), tmp.getZ(), tmp.getW());
    box_transform.mat = box_transform.quaternion.toMatrix();
}

void WheeledBoxPhysicsObject::updateStubs(void){
    for(unsigned i = 0; i < stubs.size(); i++){
        btTransform trans;
        stubs[i].body->getMotionState()->getWorldTransform(trans);

        stubs[i].transform.shift.x = trans.getOrigin().getX();
        stubs[i].transform.shift.y = trans.getOrigin().getY();
        stubs[i].transform.shift.z = trans.getOrigin().getZ();

        btQuaternion tmp = trans.getRotation();
        stubs[i].transform.quaternion = Quaternion(tmp.getX(), tmp.getY(), tmp.getZ(), tmp.getW());
        stubs[i].transform.mat = stubs[i].transform.quaternion.toMatrix();
    }
}

void WheeledBoxPhysicsObject::updateWheels(void){
    for(unsigned i = 0; i < wheels.size(); i++){
        btTransform trans;
        wheels[i].body->getMotionState()->getWorldTransform(trans);

        wheels[i].transform.shift.x = trans.getOrigin().getX();
        wheels[i].transform.shift.y = trans.getOrigin().getY();
        wheels[i].transform.shift.z = trans.getOrigin().getZ();

        btQuaternion tmp = trans.getRotation();
        wheels[i].transform.quaternion = Quaternion(tmp.getX(), tmp.getY(), tmp.getZ(), tmp.getW());
        wheels[i].transform.mat = wheels[i].transform.quaternion.toMatrix();
    }
}

void WheeledBoxPhysicsObject::setTransform(Transform transform, btDefaultMotionState *motion_state, btRigidBody *body){
    btTransform new_transform;
    new_transform.setIdentity();
    new_transform.setOrigin(btVector3(transform.shift.x, transform.shift.y, transform.shift.z));
    new_transform.setRotation(btQuaternion(transform.quaternion.x, transform.quaternion.y,
                                           transform.quaternion.z, transform.quaternion.w));

    motion_state->setWorldTransform(new_transform);
    body->setMotionState(motion_state);
}

void WheeledBoxPhysicsObject::applyImpulse(Vector3D impulse, Vector3D pos){
    btVector3 bt_impulse(impulse.x, impulse.y, impulse.z);
    btVector3 bt_pos(pos.x, pos.y, pos.z);
    box_body->applyImpulse(bt_impulse, bt_pos);
}

void WheeledBoxPhysicsObject::perturbParams(void){
    for(unsigned i = 0; i < stubs.size(); i++){
        float new_friction = stubs[i].params.friction + Common::uniformNoise(-0.1f, 0.1f);
        if(new_friction < 0.01f){
            new_friction = 0.01f;
        }
        stubs[i].body->setFriction(new_friction);
    }

    for(unsigned i = 0; i < wheels.size(); i++){
        float new_friction = wheels[i].params.friction + Common::uniformNoise(-0.1f, 0.1f);
        if(new_friction < 0.01f){
            new_friction = 0.01f;
        }
        wheels[i].body->setFriction(new_friction);

        if(wheels[i].params.damping < 1.0f){
            float new_damping = wheels[i].params.damping + Common::uniformNoise(-0.1f, 0.1f);
            if(new_damping > 1.0f){
                new_damping = 1.0f;
            }
            if(new_damping < 0.01f){
                new_damping = 0.01f;
            }

            wheels[i].body->setDamping(LINEAR_DAMPING, new_damping);
        }
    }
}