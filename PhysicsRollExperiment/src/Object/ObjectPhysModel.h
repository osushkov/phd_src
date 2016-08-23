
#ifndef _ObjectPhysModel_H_
#define _ObjectPhysModel_H_

#include "../Util/Vector3D.h"
#include "../Physics/PhysicsObject.h"
#include "../Physics/WheeledBoxPhysicsObject.h"
#include "../Visualisation/RenderObject.h"
#include "../Visualisation/WheeledBoxRenderObject.h"

class ObjectPhysModel {
  public:
      ObjectPhysModel(Vector3D box_size, float box_mass, 
                      std::vector<WheeledBoxPhysicsObject::WheelParams> wheel_params,
                      std::vector<WheeledBoxPhysicsObject::StubParams> stub_params);

    ~ObjectPhysModel();

    WheeledBoxPhysicsObject* getPhysicsObject(void);
    WheeledBoxRenderObject* getRenderObject(void);

    unsigned getId(void) const;

  private:
    Vector3D box_size;
    float box_mass;
    
    std::vector<WheeledBoxPhysicsObject::WheelParams> wheel_params;
    std::vector<WheeledBoxPhysicsObject::StubParams> stub_params;

    unsigned id;
    static unsigned generateId(void);
};

#endif
