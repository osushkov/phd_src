/*
 * MeshRenderObject.cpp
 *
 *  Created on: 02/04/2010
 *      Author: osushkov
 */

#include "MeshRenderObject.h"
#include "../Util/Common.h"

#ifdef _WIN32
#include <windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>

MeshRenderObject::MeshRenderObject(Mesh mesh) : mesh(mesh) {
    transform.mat.identity();
    transform.quaternions = Common::matrixToQuaternions(transform.mat);
    transform.shift = transform.secondary_shift = Vector3D(0.0f, 0.0f, 0.0f);
}

MeshRenderObject::~MeshRenderObject(){

}

void MeshRenderObject::setUseTexture(bool var){
    use_texture = var;
}

void MeshRenderObject::setColor(Vector3D var){
    color = var;
}

void MeshRenderObject::setTransform(Transform new_transform){
    transform = new_transform;
}

void MeshRenderObject::render(void){
    glPushMatrix();

    glTranslatef(transform.shift.x,transform.shift.y, transform.shift.z);

    glPointSize(1.0f);
    //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glEnable(GL_LIGHTING);
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

    glBegin(GL_TRIANGLES);

    for(unsigned i = 0; i < mesh.face_indices.size(); i++){
        Vector3D normal = mesh.face_normals[i];
        glNormal3f(normal.x, normal.y, normal.z);

        for(unsigned j = 0 ; j < mesh.face_indices[i].size(); j++){
            Vector3D vertex = mesh.vertices[mesh.face_indices[i][j]];
            glVertex3f(vertex.x, vertex.y, vertex.z);
        }
    }

    glEnd();

    glPopMatrix();
}
