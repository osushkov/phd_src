/*
 * SuperQuadricRenderObject.cpp
 *
 *  Created on: 28/10/2009
 *      Author: osushkov
 */

#include "SuperQuadricRenderObject.h"
#include <GL/gl.h>
#include <GL/glu.h>

#define _USE_MATH_DEFINES
#include <math.h>

SuperQuadricRenderObject::SuperQuadricRenderObject(SuperQuadric sq_shape) :
                                                   sq_shape(sq_shape) {
	id = RenderObject::getNewId();

	transform.shift = Vector3D(0.0f, 0.0f, 0.0f);
	transform.mat.identity();
}

SuperQuadricRenderObject::~SuperQuadricRenderObject(){

}

void SuperQuadricRenderObject::setUseTexture(bool var){

}

void SuperQuadricRenderObject::setColor(Vector3D var){

}

void SuperQuadricRenderObject::setTransform(Transform new_transform){
    transform = new_transform;
}

void SuperQuadricRenderObject::render(void){
    glPushMatrix();
    
    glTranslatef(transform.shift.x,transform.shift.y, transform.shift.z);

    glPointSize(1.0f);
    //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glDisable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    glDisable(GL_TEXTURE_2D);


    float m[16] = {0.0f};
    m[0] = transform.mat(0, 0);
    m[1] = transform.mat(1, 0);
    m[2] = transform.mat(2, 0);

    m[4] = transform.mat(0, 1);
    m[5] = transform.mat(1, 1);
    m[6] = transform.mat(2, 1);

    m[8] = transform.mat(0, 2);
    m[9] = transform.mat(1, 2);
    m[10] = transform.mat(2, 2);

    m[15] = 1.0f;

    glMultMatrixf(m);

    glBegin(GL_QUADS);

    const float inc = 0.1f;
    for(float u = -M_PI; u < M_PI; u += inc){
        for(float v = -M_PI/2.0f; v < M_PI/2.0f; v += inc){
            Vector3D surface_point = sq_shape.getSurfacePosition(u, v);
            Vector3D surface_normal = sq_shape.getSurfaceNormal(u, v);
            glNormal3f(surface_normal.x, surface_normal.y, surface_normal.z);
            glVertex3f(surface_point.x, surface_point.y, surface_point.z);

            surface_point = sq_shape.getSurfacePosition(u, v+inc);
            surface_normal = sq_shape.getSurfaceNormal(u, v+inc);
            glNormal3f(surface_normal.x, surface_normal.y, surface_normal.z);
            glVertex3f(surface_point.x, surface_point.y, surface_point.z);

            surface_point = sq_shape.getSurfacePosition(u+inc, v+inc);
            surface_normal = sq_shape.getSurfaceNormal(u+inc, v+inc);
            glNormal3f(surface_normal.x, surface_normal.y, surface_normal.z);
            glVertex3f(surface_point.x, surface_point.y, surface_point.z);

            surface_point = sq_shape.getSurfacePosition(u+inc, v);
            surface_normal = sq_shape.getSurfaceNormal(u+inc, v);
            glNormal3f(surface_normal.x, surface_normal.y, surface_normal.z);
            glVertex3f(surface_point.x, surface_point.y, surface_point.z);
        }
    }

    glEnd();

    glPopMatrix();
}

