/*
 * CylinderRenderObject.cpp
 *
 *  Created on: 12/07/2009
 *      Author: osushkov
 */

#include "CylinderRenderObject.h"

#ifdef _WIN32
#include <windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>

CylinderRenderObject::CylinderRenderObject(Vector3D start, Vector3D end, float radius) :
    start(start), end(end), radius(radius), use_texture(false), color(1.0f, 1.0f, 1.0f) {

    id = RenderObject::getNewId();

    quadric = gluNewQuadric();
    gluQuadricNormals(quadric, GLU_SMOOTH);

    createOrientationMatrix();
}

CylinderRenderObject::~CylinderRenderObject(){
    gluDeleteQuadric(quadric);
}

void CylinderRenderObject::setStart(Vector3D var){
    start = var;
    createOrientationMatrix();
}

void CylinderRenderObject::setEnd(Vector3D var){
    end = var;
    createOrientationMatrix();
}

void CylinderRenderObject::setRadius(float var){
    radius = var;
}

void CylinderRenderObject::setUseTexture(bool var){
    use_texture = var;
}

void CylinderRenderObject::setColor(Vector3D var){
    color = var;
}

void CylinderRenderObject::render(void){
    float adj[16];
    //matrixToOpenGLMatrix(transform, adj);

    glPushMatrix();

    glMultMatrixf(adj);
    glMultMatrixf(orientation_matrix);

    float axis_offset = 5.0f;
    glBegin(GL_LINES);
    glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(axis_offset, 0.0f, 0.0f);

    glColor4f(1.0f, 0.0f, 1.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, axis_offset, 0.0f);

    glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, axis_offset);
    glEnd();

    glColor4f(color.x, color.y, color.z, 1.0f);
    gluCylinder(quadric, radius, radius, (start - end).length(), 32, 32);

    glPopMatrix();
}

void CylinderRenderObject::createOrientationMatrix(void){
    Vector3D v1 = (end - start);
    v1.normalise();

    Vector3D v2(v1.x+8.0f, v1.y+4.0f, v1.z+2.0f);
    v2.normalise();

    Vector3D v3 = v1.crossProduct(v2);
    v2 = v1.crossProduct(v3);

    v2.normalise();
    v3.normalise();

    for(unsigned i = 0; i < 16; i++){
        orientation_matrix[i] = 0.0f;
    }

    orientation_matrix[0] = v2.x;
    orientation_matrix[1] = v2.y;
    orientation_matrix[2] = v2.z;

    orientation_matrix[4] = v3.x;
    orientation_matrix[5] = v3.y;
    orientation_matrix[6] = v3.z;

    orientation_matrix[8] = v1.x;
    orientation_matrix[9] = v1.y;
    orientation_matrix[10] = v1.z;

    orientation_matrix[15] = 1.0f;
}

void CylinderRenderObject::matrixToOpenGLMatrix(Matrix4 mat, float *result){
    for(unsigned i = 0; i < 16; i++){
        result[i] = 0.0f;
    }

    result[0] = mat(0, 0);
    result[1] = mat(1, 0);
    result[2] = mat(2, 0);

    result[4] = mat(0, 1);
    result[5] = mat(1, 1);
    result[6] = mat(2, 1);

    result[8] = mat(0, 2);
    result[9] = mat(1, 2);
    result[10] = mat(2, 2);

    result[12] = mat(0, 3);
    result[13] = mat(1, 3);
    result[14] = mat(2, 3);
    result[15] = 1.0f;
}
