/*
 * BoxRenderObject.cpp
 *
 *  Created on: 03/03/2010
 *      Author: osushkov
 */

#include "BoxRenderObject.h"
#include "TextureManager.h"

#ifdef _WIN32
#include <windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>

BoxRenderObject::BoxRenderObject(float width, float height, float depth) :
    width(width/2.0f), height(height/2.0f), depth(depth/2.0f), use_texture(false) {

    id = RenderObject::getNewId();
    transform.mat.identity();
    transform.shift = Vector3D(0.0f, 0.0f, 0.0f);

}

BoxRenderObject::~BoxRenderObject(){

}

void BoxRenderObject::setUseTexture(bool var){
    use_texture = var;
}

void BoxRenderObject::setTextureIDs(std::vector<unsigned> _texture_ids){
    texture_ids = _texture_ids;
}



void BoxRenderObject::setColor(Vector3D var){
    color = var;
}

void BoxRenderObject::render(void){
    glPushMatrix();

    float adj[16];
    RenderObject::transformToOpenGLMatrix(adj);

    glMultMatrixf(adj);

    glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
    //glEnable(GL_LIGHTING);
    //glEnable(GL_LIGHT0);

    if(use_texture){
        glEnable(GL_TEXTURE_2D);
    }

    if(use_texture){ TextureManager::instance().bindTexture(texture_ids[0]); }
    glBegin(GL_QUADS);
    glNormal3f(0.0f, 0.0f, 1.0f);
    glTexCoord2f(0.0f, 0.0f); glVertex3f(-width, -height, depth);
    glTexCoord2f(0.0f, 1.0f); glVertex3f(-width, height, depth);
    glTexCoord2f(1.0f, 1.0f); glVertex3f(width, height, depth);
    glTexCoord2f(1.0f, 0.0f); glVertex3f(width, -height, depth);
    glEnd();

    if(use_texture){ TextureManager::instance().bindTexture(texture_ids[1]); }
    glBegin(GL_QUADS);
    glNormal3f(0.0f, 0.0f, -1.0f);
    glTexCoord2f(0.0f, 0.0f); glVertex3f(-width, -height, -depth);
    glTexCoord2f(0.0f, 1.0f); glVertex3f(-width, height, -depth);
    glTexCoord2f(1.0f, 1.0f); glVertex3f(width, height, -depth);
    glTexCoord2f(1.0f, 0.0f); glVertex3f(width, -height, -depth);
    glEnd();

    if(use_texture){ TextureManager::instance().bindTexture(texture_ids[2]); }
    glBegin(GL_QUADS);
    glNormal3f(1.0f, 0.0f, .0f);
    glTexCoord2f(0.0f, 0.0f); glVertex3f(width, -height, -depth);
    glTexCoord2f(0.0f, 1.0f); glVertex3f(width, -height, depth);
    glTexCoord2f(1.0f, 1.0f); glVertex3f(width, height, depth);
    glTexCoord2f(1.0f, 0.0f); glVertex3f(width, height, -depth);
    glEnd();

    if(use_texture){ TextureManager::instance().bindTexture(texture_ids[3]); }
    glBegin(GL_QUADS);
    glNormal3f(-1.0f, 0.0f, .0f);
    glTexCoord2f(0.0f, 0.0f); glVertex3f(-width, -height, -depth);
    glTexCoord2f(0.0f, 1.0f); glVertex3f(-width, -height, depth);
    glTexCoord2f(1.0f, 1.0f); glVertex3f(-width, height, depth);
    glTexCoord2f(1.0f, 0.0f); glVertex3f(-width, height, -depth);
    glEnd();

    if(use_texture){ TextureManager::instance().bindTexture(texture_ids[4]); }
    glBegin(GL_QUADS);
    glNormal3f(0.0f, 1.0f, .0f);
    glTexCoord2f(0.0f, 0.0f); glVertex3f(-width, height, -depth);
    glTexCoord2f(0.0f, 1.0f); glVertex3f(-width, height, depth);
    glTexCoord2f(1.0f, 1.0f); glVertex3f(width, height, depth);
    glTexCoord2f(1.0f, 0.0f); glVertex3f(width, height, -depth);
    glEnd();

    if(use_texture){ TextureManager::instance().bindTexture(texture_ids[5]); }
    glBegin(GL_QUADS);
    glNormal3f(0.0f, -1.0f, .0f);
    glTexCoord2f(0.0f, 0.0f); glVertex3f(-width, -height, -depth);
    glTexCoord2f(0.0f, 1.0f); glVertex3f(-width, -height, depth);
    glTexCoord2f(1.0f, 1.0f); glVertex3f(width, -height, depth);
    glTexCoord2f(1.0f, 0.0f); glVertex3f(width, -height, -depth);
    glEnd();

    if(use_texture){
        glDisable(GL_TEXTURE_2D);
    }

    glPopMatrix();
}
