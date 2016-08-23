/*
 * BoxRenderObject.h
 *
 *  Created on: 03/03/2010
 *      Author: osushkov
 */

#ifndef BOXRENDEROBJECT_H_
#define BOXRENDEROBJECT_H_

#include "RenderObject.h"
#include "../Util/Vector3D.h"

#ifdef _WIN32
#include <windows.h>
#endif

#include <GL/glu.h>


class BoxRenderObject : public RenderObject {
  public:
    BoxRenderObject(float width, float height, float depth);
    ~BoxRenderObject();

    void setUseTexture(bool var);
    void setTextureIDs(std::vector<unsigned> _texture_ids);
    void setColor(Vector3D var);

    void render(void);

  private:
    float width, height, depth;

    bool use_texture;
    Vector3D color;
    std::vector<unsigned> texture_ids;
};

#endif /* BOXRENDEROBJECT_H_ */
