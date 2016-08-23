/*
 * BoxRenderObject.cpp
 *
 *  Created on: 03/03/2010
 *      Author: osushkov
 */

#include "BoxRenderObject.h"

#ifdef _WIN32
#include <windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>

BoxRenderObject::BoxRenderObject(float width, float height, float depth) :
    width(width/2.0f), height(height/2.0f), depth(depth/2.0f) {

    id = RenderObject::getNewId();
    transform.mat.identity();
    transform.shift = Vector3D(0.0f, 0.0f, 0.0f);

}

BoxRenderObject::~BoxRenderObject(){

}

void BoxRenderObject::setUseTexture(bool var){
    use_texture = var;
}

void BoxRenderObject::setColor(Vector3D var){
    color = var;
}

void BoxRenderObject::render(void){
    float adj[16];
    transformToOpenGLMatrix(adj);

    glPushMatrix();

    glMultMatrixf(adj);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    glBegin(GL_QUADS);

    glNormal3f(0.0f, 0.0f, 1.0f);
    glVertex3f(-width, -height, depth);
    glVertex3f(-width, height, depth);
    glVertex3f(width, height, depth);
    glVertex3f(width, -height, depth);

    glNormal3f(0.0f, 0.0f, -1.0f);
    glVertex3f(-width, -height, -depth);
    glVertex3f(-width, height, -depth);
    glVertex3f(width, height, -depth);
    glVertex3f(width, -height, -depth);


    glNormal3f(1.0f, 0.0f, .0f);
    glVertex3f(width, -height, -depth);
    glVertex3f(width, -height, depth);
    glVertex3f(width, height, depth);
    glVertex3f(width, height, -depth);

    glNormal3f(-1.0f, 0.0f, .0f);
    glVertex3f(-width, -height, -depth);
    glVertex3f(-width, -height, depth);
    glVertex3f(-width, height, depth);
    glVertex3f(-width, height, -depth);


    glNormal3f(0.0f, 1.0f, .0f);
    glVertex3f(-width, height, -depth);
    glVertex3f(-width, height, depth);
    glVertex3f(width, height, depth);
    glVertex3f(width, height, -depth);

    glNormal3f(0.0f, -1.0f, .0f);
    glVertex3f(-width, -height, -depth);
    glVertex3f(-width, -height, depth);
    glVertex3f(width, -height, depth);
    glVertex3f(width, -height, -depth);

    glEnd();

    glPopMatrix();
}

void BoxRenderObject::transformToOpenGLMatrix(float *result){
    for(unsigned i = 0; i < 16; i++){
        result[i] = 0.0f;
    }

    result[0] = transform.mat(0, 0);
    result[1] = transform.mat(1, 0);
    result[2] = transform.mat(2, 0);

    result[4] = transform.mat(0, 1);
    result[5] = transform.mat(1, 1);
    result[6] = transform.mat(2, 1);

    result[8] = transform.mat(0, 2);
    result[9] = transform.mat(1, 2);
    result[10] = transform.mat(2, 2);

    result[12] = transform.shift.x;
    result[13] = transform.shift.y;
    result[14] = transform.shift.z;

    result[15] = 1.0f;

}
