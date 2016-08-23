
#include "PlaneRenderObject.h"

PlaneRenderObject::PlaneRenderObject(Vector3D corner, Vector3D edge0, Vector3D edge1, float grid_res) :
    corner(corner), edge0(edge0), edge1(edge1), grid_res(grid_res) {

    id = RenderObject::getNewId();
    
}

PlaneRenderObject::~PlaneRenderObject(){

}

void PlaneRenderObject::setUseTexture(bool var){

}

void PlaneRenderObject::setColor(Vector3D var){
    color = var;
}

void PlaneRenderObject::render(void){
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);
    glPushMatrix();

    glColor4f(color.x, color.y, color.z, 1.0f);

    unsigned num_lines0 = edge0.length()/grid_res;
    unsigned num_lines1 = edge1.length()/grid_res;

    glBegin(GL_LINES);
    for(unsigned i = 0; i <= num_lines0; i++){
        Vector3D start = corner + (float)i/(float)num_lines0 * edge0;
        Vector3D end = corner + edge1 + (float)i/(float)num_lines0 * edge0;
 
        glVertex3f(start.x, start.y, start.z);
        glVertex3f(end.x, end.y, end.z);
    }

    for(unsigned i = 0; i <= num_lines1; i++){
        Vector3D start = corner + (float)i/(float)num_lines1 * edge1;
        Vector3D end = corner + edge0 + (float)i/(float)num_lines1 * edge1;

        glVertex3f(start.x, start.y, start.z);
        glVertex3f(end.x, end.y, end.z);
    }

    glEnd();

    glPopMatrix();
}