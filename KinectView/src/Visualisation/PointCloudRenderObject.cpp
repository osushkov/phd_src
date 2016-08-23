/*
 * PointCloudRenderObject.cpp
 *
 *  Created on: 12/07/2009
 *      Author: osushkov
 */

#include "PointCloudRenderObject.h"

#ifdef _WIN32
#include <windows.h>
#endif

#include <GL/gl.h>


PointCloudRenderObject::PointCloudRenderObject() :
    use_texture(false), color(1.0f, 1.0f, 1.0f), lock(1) {
    id = RenderObject::getNewId();

}

PointCloudRenderObject::PointCloudRenderObject(bool use_texture, Vector3D color) :
    use_texture(use_texture), color(color), lock(1) {

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

void PointCloudRenderObject::updatePoints(const std::vector<PointCloudPoint> &new_points){
    lock.wait();
    points = new_points;
    lock.signal();
}

void PointCloudRenderObject::setUseTexture(bool var){
    use_texture = var;
}

void PointCloudRenderObject::setColor(Vector3D var){
    color = var;
}

void PointCloudRenderObject::render(void){
    glDisable(GL_LIGHTING);
    glDisable(GL_LIGHT0);

    glColor4f(color.x, color.y, color.z, 1.0f);

    std::vector<PointCloudPoint> cur_points;
    lock.wait();
    cur_points = points;
    lock.signal();

    glPushMatrix();

    glBegin(GL_POINTS);
    for(unsigned i = 0; i < cur_points.size(); i++){
        glColor4f(cur_points[i].color.x, cur_points[i].color.y, cur_points[i].color.z, 1.0f);
        glVertex3f(cur_points[i].pos.x, cur_points[i].pos.y, cur_points[i].pos.z);
    }
    

    glEnd();

/*
    glBegin(GL_QUADS);
    for(unsigned i = 0; i < cur_points.size(); i++){
        glColor4f(cur_points[i].color.x, cur_points[i].color.y, cur_points[i].color.z, 1.0f);
        renderCube(cur_points[i].size/2.0f, cur_points[i].pos);
    }
    glEnd();
*/
    glPopMatrix();
}

void PointCloudRenderObject::renderCube(float size, Vector3D pos){   
    glNormal3f(0.0f, 0.0f, 1.0f);
    glVertex3f(-size+pos.x, -size+pos.y, size+pos.z);
    glVertex3f(-size+pos.x, size+pos.y, size+pos.z);
    glVertex3f(size+pos.x, size+pos.y, size+pos.z);
    glVertex3f(size+pos.x, -size+pos.y, size+pos.z);

    glNormal3f(0.0f, 0.0f, -1.0f);
    glVertex3f(-size+pos.x, -size+pos.y, -size+pos.z);
    glVertex3f(-size+pos.x, size+pos.y, -size+pos.z);
    glVertex3f(size+pos.x, size+pos.y, -size+pos.z);
    glVertex3f(size+pos.x, -size+pos.y, -size+pos.z);

    glNormal3f(1.0f, 0.0f, 0.0f);
    glVertex3f(size+pos.x, -size+pos.y, -size+pos.z);
    glVertex3f(size+pos.x, -size+pos.y, size+pos.z);
    glVertex3f(size+pos.x, size+pos.y, size+pos.z);
    glVertex3f(size+pos.x, size+pos.y, -size+pos.z);

    glNormal3f(-1.0f, 0.0f, 0.0f);
    glVertex3f(-size+pos.x, -size+pos.y, -size+pos.z);
    glVertex3f(-size+pos.x, -size+pos.y, size+pos.z);
    glVertex3f(-size+pos.x, size+pos.y, size+pos.z);
    glVertex3f(-size+pos.x, size+pos.y, -size+pos.z);

    glNormal3f(0.0f, 1.0f, 0.0f);
    glVertex3f(-size+pos.x, size+pos.y, -size+pos.z);
    glVertex3f(-size+pos.x, size+pos.y, size+pos.z);
    glVertex3f(size+pos.x, size+pos.y, size+pos.z);
    glVertex3f(size+pos.x, size+pos.y, -size+pos.z);

    glNormal3f(0.0f, -1.0f, 0.0f);
    glVertex3f(-size+pos.x, -size+pos.y, -size+pos.z);
    glVertex3f(-size+pos.x, -size+pos.y, size+pos.z);
    glVertex3f(size+pos.x, -size+pos.y, size+pos.z);
    glVertex3f(size+pos.x, -size+pos.y, -size+pos.z);
}