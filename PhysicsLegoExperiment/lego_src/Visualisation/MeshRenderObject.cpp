/*
 * MeshRenderObject.cpp
 *
 *  Created on: 05/03/2010
 *      Author: osushkov
 */

#include "MeshRenderObject.h"

#ifdef _WIN32
#include <windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>

MeshRenderObject::MeshRenderObject(Mesh mesh) : mesh(mesh) {

    id = RenderObject::getNewId();
    transform.mat.identity();
    transform.shift = Vector3D(0.0f, 0.0f, 0.0f);
}

MeshRenderObject::~MeshRenderObject(){

}

void MeshRenderObject::setUseTexture(bool var){
    use_texture = var;
}

void MeshRenderObject::setColor(Vector3D var){
    color = var;
}

void MeshRenderObject::render(void){
    float adj[16];
    transformToOpenGLMatrix(adj);

    glPushMatrix();

    glMultMatrixf(adj);

    //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    assert(mesh.face_indices.size() == mesh.face_normals.size());
    glBegin(GL_TRIANGLES);
    for(unsigned i = 0; i < mesh.face_indices.size(); i++){
        assert(mesh.face_indices[i].size() == 3);
        assert(i < mesh.face_normals.size());
        glNormal3f(mesh.face_normals[i].x, mesh.face_normals[i].y, mesh.face_normals[i].z);
        for(unsigned j = 0; j < 3; j++){
            unsigned index = mesh.face_indices[i].at(j);
            assert(index < mesh.vertices.size());
            glVertex3f(mesh.vertices[index].x, mesh.vertices[index].y, mesh.vertices[index].z);
        }
    }
    glEnd();

    glPopMatrix();
}

void MeshRenderObject::transformToOpenGLMatrix(float *result){
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
