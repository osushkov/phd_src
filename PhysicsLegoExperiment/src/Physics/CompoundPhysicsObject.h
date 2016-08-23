
#ifndef _CompoundPhysicsObject_H_
#define _CompoundPhysicsObject_H_

#include "PhysicsObject.h"
#include "PhysicsWorld.h"
#include "../Util/Vector3D.h"

#include <vector>

class CompoundPhysicsObject {
  public:
    CompoundPhysicsObject(std::vector<PhysicsObject*> sub_objects, std::vector<Vector3D> rel_positions);
    ~CompoundPhysicsObject();

    void addToWorld(PhysicsWorld *world);
    unsigned getId(void);

  private:
    std::vector<PhysicsObject*> sub_objects;
    std::vector<btGeneric6DofConstraint*> constraints;
    std::vector<Vector3D> rel_positions;

    unsigned id;
};

#endif
