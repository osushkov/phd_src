
/*
 * LightBeamRenderObject.cpp
 *
 *  Created on: 03/03/2010
 *      Author: osushkov
 */

#include "LightBeamRenderObject.h"

#ifdef _WIN32
#include <windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>

#define _USE_MATH_DEFINES
#include <math.h>

LightBeamRenderObject::LightBeamRenderObject(Vector3D position, float radius) :
    position(position), radius(radius), color(1.0f, 0.0f, 0.0f) {

    id = RenderObject::getNewId();

}

LightBeamRenderObject::~LightBeamRenderObject(){

}

void LightBeamRenderObject::setUseTexture(bool var){

}

void LightBeamRenderObject::setColor(Vector3D var){
    color = var;
}

void LightBeamRenderObject::render(void){

    glPushMatrix();

    glDisable(GL_LIGHTING);
    glDisable(GL_LIGHT0);

    glBegin(GL_TRIANGLE_FAN);
    glColor4f(color.x, color.y, color.z, 1.0f);

    glNormal3f(0.0f, 0.0f, 1.0f);
    glVertex3f(position.x, position.y, position.z);

    const unsigned num_segments = 30;
    for(unsigned i = 0; i <= num_segments; i++){
        float theta = (float)i/(float)num_segments * 2.0f * M_PI;
        Vector3D offset(radius * cos(theta), radius * sin(theta), 0.0f);
        glVertex3f(position.x + offset.x, position.y + offset.y, position.z + offset.z);
    }

    glEnd();

    glPopMatrix();
}

