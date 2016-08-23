/*
 * SuperQuadricRenderObject.h
 *
 *  Created on: 28/10/2009
 *      Author: osushkov
 */

#ifndef SUPERQUADRICRENDEROBJECT_H_
#define SUPERQUADRICRENDEROBJECT_H_

#include "RenderObject.h"
#include <vector>
#include "../Util/Vector3D.h"
#include "../SuperQuadric.h"

class SuperQuadricRenderObject : public RenderObject {
  public:
    SuperQuadricRenderObject(SuperQuadric sq_shape);
    ~SuperQuadricRenderObject();

    void setUseTexture(bool var);
    void setColor(Vector3D var);
    void setTransform(Transform new_transform);

    void render(void);

  private:
    const SuperQuadric sq_shape;

    bool use_texture;
    Vector3D color;

    Transform transform;
};



#endif /* SUPERQUADRICRENDEROBJECT_H_ */
