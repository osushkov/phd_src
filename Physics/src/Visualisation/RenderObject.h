/*
 * RenderObject.h
 *
 *  Created on: 12/07/2009
 *      Author: osushkov
 */

#ifndef RENDEROBJECT_H_
#define RENDEROBJECT_H_

#include "../Util/Matrix.h"
#include "../Util/Transform.h"

#ifdef _WIN32
#include <windows.h>
#endif

class RenderObject {
  public:
    static unsigned getNewId(void){
        static unsigned cur_id;
        return cur_id++;
    }

    virtual ~RenderObject(){}
    virtual void render(void) = 0;

    virtual void setTransform(Transform t){
        transform = t;
    }

    virtual unsigned getId(void){
        return id;
    }

  protected:
    unsigned id;

    Transform transform;

};


#endif /* RENDEROBJECT_H_ */
