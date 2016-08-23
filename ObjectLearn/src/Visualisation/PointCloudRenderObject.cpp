/*
 * PointCloudRenderObject.cpp
 *
 *  Created on: 12/07/2009
 *      Author: osushkov
 */

#include "PointCloudRenderObject.h"
#include <GL/gl.h>


PointCloudRenderObject::PointCloudRenderObject() :
    use_texture(false), color(1.0f, 1.0f, 1.0f) {
    id = RenderObject::getNewId();

}

PointCloudRenderObject::PointCloudRenderObject(bool use_texture, Vector3D color) :
    use_texture(use_texture), color(color) {

}

PointCloudRenderObject::~PointCloudRenderObject(){

}

void PointCloudRenderObject::addPoints(const std::vector<PointCloudPoint> &new_points){
    for(unsigned i = 0; i < new_points.size(); i++){
        points.push_back(new_points[i]);
    }
}

void PointCloudRenderObject::clearPoints(void){
    points.clear();
}

void PointCloudRenderObject::setUseTexture(bool var){
    use_texture = var;
}

void PointCloudRenderObject::setColor(Vector3D var){
    color = var;
}

void PointCloudRenderObject::render(void){
    glPushMatrix();

    glDisable(GL_LIGHTING);
    glDisable(GL_LIGHT0);

    glPointSize(2.0f);
    glColor4f(color.x, color.y, color.z, 1.0f);

    if(use_texture){
        glEnable(GL_TEXTURE_2D);
        glBegin(GL_QUADS);
    }
    else{
        glDisable(GL_TEXTURE_2D);
        glBegin(GL_POINTS);
    }

    for(unsigned i = 0; i < points.size(); i++){
        renderPoint(points[i]);
    }

    glEnd();

    glPopMatrix();
}

void PointCloudRenderObject::renderPoint(const PointCloudPoint &point){
    if(use_texture){
        // TODO
    	assert(false);
    }
    else{
        if(point.color.x > 0.01f || point.color.y > 0.01f || point.color.z > 0.0f){
            glColor3f(point.color.x, point.color.y, point.color.z);
        }
        glVertex3f(point.pos.x, point.pos.y, point.pos.z);
    }
}
