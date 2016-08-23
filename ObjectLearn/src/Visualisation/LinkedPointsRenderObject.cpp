/*
 * LinkedPointsRenderObject.cpp
 *
 *  Created on: 16/07/2009
 *      Author: osushkov
 */

#include "LinkedPointsRenderObject.h"
#include <GL/gl.h>


LinkedPointsRenderObject::LinkedPointsRenderObject() :
    use_texture(false), color(1.0f, 1.0f, 1.0f) {
    id = RenderObject::getNewId();

}

LinkedPointsRenderObject::LinkedPointsRenderObject(bool use_texture, Vector3D color) :
    use_texture(use_texture), color(color) {

}

LinkedPointsRenderObject::~LinkedPointsRenderObject(){

}

void LinkedPointsRenderObject::addPoints
(const std::vector<std::pair<PointCloudPoint,PointCloudPoint> > &new_points){
    for(unsigned i = 0; i < new_points.size(); i++){
        points.push_back(new_points[i]);
    }
}

void LinkedPointsRenderObject::clearPoints(void){
    points.clear();
}

void LinkedPointsRenderObject::setUseTexture(bool var){
    use_texture = var;
}

void LinkedPointsRenderObject::setColor(Vector3D var){
    color = var;
}

void LinkedPointsRenderObject::render(void){
    glPushMatrix();

    glColor4f(color.x, color.y, color.z, 1.0f);

    if(use_texture){
        glEnable(GL_TEXTURE_2D);
    }
    else{
        glDisable(GL_TEXTURE_2D);
    }

    for(unsigned i = 0; i < points.size(); i++){
        renderPoint(points[i]);
    }

    glPopMatrix();
}

void LinkedPointsRenderObject::renderPoint(const std::pair<PointCloudPoint,PointCloudPoint> &point){
    if(use_texture){
        // TODO
    }
    else {
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        glBegin(GL_LINES);
        glVertex3f(point.first.pos.x, point.first.pos.y, point.first.pos.z);
        glVertex3f(point.second.pos.x, point.second.pos.y, point.second.pos.z);
        glEnd();

        glBegin(GL_POINTS);
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        glVertex3f(point.first.pos.x, point.first.pos.y, point.first.pos.z);

        glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
        glVertex3f(point.second.pos.x, point.second.pos.y, point.second.pos.z);
        glEnd();
    }
}
