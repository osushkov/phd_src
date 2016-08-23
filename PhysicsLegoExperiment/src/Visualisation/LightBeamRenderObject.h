
#ifndef _LightBeamRenderObject_H_
#define _LightBeamRenderObject_H_

#include "RenderObject.h"
#include "../Util/Vector3D.h"

#ifdef _WIN32
#include <windows.h>
#endif

#include <GL/glu.h>

class LightBeamRenderObject : public RenderObject {
  public:
    LightBeamRenderObject(Vector3D position, float radius);
    ~LightBeamRenderObject();

    void setUseTexture(bool var);
    void setColor(Vector3D var);

    void render(void);

  private:
    Vector3D position;
    float radius;

    Vector3D color;
};

#endif
