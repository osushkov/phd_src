/*
 * PlaneRenderObject.h
 *
 *  Created on: 23/07/2010
 *      Author: osushkov
 */

#ifndef PLANERENDEROBJECT_H_
#define PLANERENDEROBJECT_H_

#include "RenderObject.h"
#include "../Util/Vector3D.h"

#include <GL/glu.h>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif


class PlaneRenderObject : public RenderObject {
  public:
	PlaneRenderObject(Vector3D corner, Vector3D edge0, Vector3D edge1, float grid_res);
    ~PlaneRenderObject();

    void setUseTexture(bool var);
    void setColor(Vector3D var);

    void render(void);

  private:
    Vector3D corner;
    Vector3D edge0;
    Vector3D edge1;
    float grid_res;

    Vector3D color;
};

#endif /* PLANERENDEROBJECT_H_ */
