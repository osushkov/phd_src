/*
 * MeshRenderObject.h
 *
 *  Created on: 05/03/2010
 *      Author: osushkov
 */

#ifndef MESHRENDEROBJECT_H_
#define MESHRENDEROBJECT_H_

#include "RenderObject.h"
#include "../Util/Vector3D.h"
#include "../Mesh.h"


class MeshRenderObject : public RenderObject {
  public:
    MeshRenderObject(Mesh mesh);
    ~MeshRenderObject();

    void setUseTexture(bool var);
    void setColor(Vector3D var);

    void render(void);

  private:
    Mesh mesh;

    bool use_texture;
    Vector3D color;

    void transformToOpenGLMatrix(float *result);
};

#endif /* MESHRENDEROBJECT_H_ */
