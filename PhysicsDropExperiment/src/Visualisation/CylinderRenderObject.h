/*
 * CylinderRenderObject.h
 *
 *  Created on: 12/07/2009
 *      Author: osushkov
 */

#ifndef CYLINDERRENDEROBJECT_H_
#define CYLINDERRENDEROBJECT_H_

#include "RenderObject.h"
#include "../Util/Vector3D.h"
#include <GL/glu.h>


class CylinderRenderObject : public RenderObject {
  public:
    CylinderRenderObject(Vector3D start, Vector3D end, float radius);
    ~CylinderRenderObject();

    void setStart(Vector3D var);
    void setEnd(Vector3D var);
    void setRadius(float var);

    void setUseTexture(bool var);
    void setColor(Vector3D var);

    void render(void);

  private:
    Vector3D start, end;
    float radius;

    GLUquadricObj *quadric;
    float orientation_matrix[16];

    bool use_texture;
    Vector3D color;


    void createOrientationMatrix(void);
    void matrixToOpenGLMatrix(Matrix4 mat, float *result);
};

#endif /* CYLINDERRENDEROBJECT_H_ */
