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

BoxRenderObject::BoxRenderObject(float width, float height, float depth, std::vector<Vector3D> sensor_pos, std::vector<Vector3D> sensor_dir) :
    width(width/2.0f), height(height/2.0f), depth(depth/2.0f), sensor_pos(sensor_pos), sensor_dir(sensor_dir) {

    id = RenderObject::getNewId();
    transform.mat.identity();
    transform.shift = Vector3D(0.0f, 0.0f, 0.0f);

    quadratic = gluNewQuadric();
    gluQuadricNormals(quadratic, GLU_SMOOTH);
    gluQuadricTexture(quadratic, GL_TRUE);
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

    glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
    for(unsigned i = 0; i < sensor_pos.size(); i++){
        glPushMatrix();
        glTranslatef(sensor_pos[i].x, sensor_pos[i].y, sensor_pos[i].z);
        gluSphere(quadratic, 1.0f , 32, 32);

        const float length = 4.0f;
        glDisable(GL_LIGHTING);
        glBegin(GL_LINES);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(length*sensor_dir[i].x, length*sensor_dir[i].y, length*sensor_dir[i].z);
        glEnd();
        glEnable(GL_LIGHTING);

        glPopMatrix();
    }

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
