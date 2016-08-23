/*
 * WheeledBoxPhysicsObject.h
 *
 *  Created on: 05/03/2010
 *      Author: osushkov
 */

#ifndef WHEELEDBOXPHYSICSOBJECT_H_
#define WHEELEDBOXPHYSICSOBJECT_H_

#include "../Util/Vector3D.h"
#include "../Util/Transform.h"
#include <btBulletDynamicsCommon.h>

class WheeledBoxPhysicsObject {
  public:
    struct WheelParams {
        Vector3D position;
        Vector3D axle;

        float radius;
        float width;

        float mass;
        float friction;
        float damping;
    };

    struct StubParams {
        Vector3D position;
        Vector3D size;

        float mass;
        float friction;
    }; 

    WheeledBoxPhysicsObject(float box_mass, 
                            Vector3D box_size, 
                            std::vector<WheelParams> wheel_params,
                            std::vector<StubParams> stub_params, 
                            Transform start_pose);

    ~WheeledBoxPhysicsObject();

    void update(void);
    void addToWorld(btDiscreteDynamicsWorld *world);
    void removeFromWorld(btDiscreteDynamicsWorld *world);

    void moveTo(Transform box_transform);

    float getLowestSurfacePoint(Quaternion q);
    std::vector<Transform> getStubTransforms(void);
    std::vector<Transform> getWheelTransforms(void);
    Transform getBoxTransform(void);
    float getBoxBottomOffset(void);

    void activate(void);
    bool isActive(void);

    unsigned getId(void) const;
    void applyImpulse(Vector3D impulse, Vector3D pos);

    void perturbParams(void);

  private:

    struct Wheel {
        WheelParams params;

        btDefaultMotionState* motion_state;
        btCollisionShape* shape;
        btRigidBody* body;
        btHingeConstraint* joint;
        Transform transform;
    };

    struct Stub {
        StubParams params;

        btDefaultMotionState* motion_state;
        btCollisionShape* shape;
        btRigidBody* body;
        btGeneric6DofConstraint* joint;
        Transform transform;
    };
    
    float box_mass;
    Vector3D box_size;

    btDefaultMotionState* box_motion_state;
    btCollisionShape* box_shape;
    btRigidBody* box_body;
    Transform box_transform;

    std::vector<Wheel> wheels; 
    std::vector<Stub> stubs;

    unsigned id;

    void buildStubConstraint(Stub &stub);
    void buildWheelConstraints(Wheel &wheel);

    void updateBox(void);
    void updateStubs(void);
    void updateWheels(void);

    void setTransform(Transform transform, btDefaultMotionState *motion_state, btRigidBody *body);
};


#endif /* WHEELEDBOXPHYSICSOBJECT_H_ */
