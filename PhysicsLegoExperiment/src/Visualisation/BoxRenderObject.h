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
#include <vector>


class BoxRenderObject : public RenderObject {
  public:
    BoxRenderObject(float width, float height, float depth, std::vector<Vector3D> sensor_pos, std::vector<Vector3D> sensor_dir);
    ~BoxRenderObject();

    void setUseTexture(bool var);
    void setColor(Vector3D var);

    void render(void);

  private:
    float width, height, depth;
    std::vector<Vector3D> sensor_pos, sensor_dir;

    GLUquadricObj *quadratic;

    bool use_texture;
    Vector3D color;

    void transformToOpenGLMatrix(float *result);
};

#endif /* BOXRENDEROBJECT_H_ */
