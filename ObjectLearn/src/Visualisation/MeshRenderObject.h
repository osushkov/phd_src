/*
 * MeshRenderObject.h
 *
 *  Created on: 02/04/2010
 *      Author: osushkov
 */

#ifndef MESHRENDEROBJECT_H_
#define MESHRENDEROBJECT_H_


#include "RenderObject.h"
#include <vector>
#include "../Util/Vector3D.h"
#include "../Util/Transform.h"
#include "../Mesh.h"

class MeshRenderObject : public RenderObject {
  public:
    MeshRenderObject(Mesh mesh);
    ~MeshRenderObject();

    void setUseTexture(bool var);
    void setColor(Vector3D var);
    void setTransform(Transform new_transform);

    void render(void);

  private:
    const Mesh mesh;

    bool use_texture;
    Vector3D color;

    Transform transform;
};



#endif /* MESHRENDEROBJECT_H_ */
