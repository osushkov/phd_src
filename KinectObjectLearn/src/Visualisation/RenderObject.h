/*
 * RenderObject.h
 *
 *  Created on: 12/07/2009
 *      Author: osushkov
 */

#ifndef RENDEROBJECT_H_
#define RENDEROBJECT_H_

#include "../Util/Matrix.h"

class RenderObject {
  public:
    static unsigned getNewId(void);

    virtual ~RenderObject(){}
    virtual void render(void) = 0;
    virtual void setTransform(Matrix4 t){
        transform = t;
    }

    virtual unsigned getId(void){
        return id;
    }

  protected:
    unsigned id;
    Matrix4 transform;
};


#endif /* RENDEROBJECT_H_ */
