
#ifndef _ObjectPhysModel_H_
#define _ObjectPhysModel_H_

#include "../SuperQuadric.h"
#include "../Util/Vector3D.h"
#include "../Physics/PhysicsObject.h"
#include "../Visualisation/RenderObject.h"

class ObjectPhysModel {
  public:
    ObjectPhysModel();
    ObjectPhysModel(SuperQuadric shape, float mass, Vector3D cog, float c_friction);
    ~ObjectPhysModel();

    PhysicsObject* getPhysicsObject(void);
    RenderObject* getRenderObject(void);

    unsigned getId(void) const;

  private:
    SuperQuadric shape;
    float mass;
    Vector3D cog;
    float c_friction;

    unsigned id;
    static unsigned generateId(void);
};

#endif
