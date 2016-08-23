
#include "WheeledBoxRenderObject.h"


#define _USE_MATH_DEFINES
#include <math.h>

WheeledBoxRenderObject::WheeledBoxRenderObject(Vector3D box_size, std::vector<Vector3D> stub_sizes, 
                                               std::vector<std::pair<float,float> > wheel_sizes) :
    box_size(box_size), stub_sizes(stub_sizes), wheel_sizes(wheel_sizes) {
   
    id = RenderObject::getNewId();

    box_transform = identityTransform();
    box_sq = SuperQuadric(0.1, 0.1, box_size.x, box_size.y, box_size.z);

    for(unsigned i = 0; i < stub_sizes.size(); i++){
        stub_transforms.push_back(identityTransform());
        stub_sqs.push_back(SuperQuadric(0.1, 0.1, stub_sizes[i].x, stub_sizes[i].y, stub_sizes[i].z));
    }

    for(unsigned i = 0; i < wheel_sizes.size(); i++){
        wheel_transforms.push_back(identityTransform());
        wheel_sqs.push_back(SuperQuadric(0.1, 1.0, wheel_sizes[i].second, wheel_sizes[i].second, wheel_sizes[i].first/2.0f));
    }
}


WheeledBoxRenderObject::~WheeledBoxRenderObject(){

}

void WheeledBoxRenderObject::setUseTexture(bool var){

}

void WheeledBoxRenderObject::setColor(Vector3D var){

}

void WheeledBoxRenderObject::render(void){
    glPushMatrix();

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    renderBox();
    renderStubs();
    renderWheels();

    glDisable(GL_LIGHTING);

    glPopMatrix();
}

void WheeledBoxRenderObject::setBoxTransform(Transform transform){
    box_transform = transform;
}

void WheeledBoxRenderObject::setStubTransforms(std::vector<Transform> transforms){
    stub_transforms = transforms;
}

void WheeledBoxRenderObject::setWheelTransforms(std::vector<Transform> transforms){
    wheel_transforms = transforms;
}

void WheeledBoxRenderObject::renderBox(void){
    glPushMatrix();

    applyTransform(box_transform);
    renderSuperQuadric(box_sq);

    glPopMatrix();
}   

void WheeledBoxRenderObject::renderStubs(void){
    for(unsigned i = 0; i < stub_sqs.size(); i++){
        glPushMatrix();

        applyTransform(stub_transforms[i]);
        renderSuperQuadric(stub_sqs[i]);

        glPopMatrix();
    }
}

void WheeledBoxRenderObject::renderWheels(void){
    for(unsigned i = 0; i < wheel_sqs.size(); i++){
        glPushMatrix();

        applyTransform(wheel_transforms[i]);
        glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
        renderSuperQuadric(wheel_sqs[i]);

        glPopMatrix();
    }
}

void WheeledBoxRenderObject::renderSuperQuadric(SuperQuadric sq){
    glBegin(GL_QUADS);

    const float inc = 0.5f;
    for(float u = -(float)M_PI; u < (float)M_PI; u += inc){
        for(float v = -(float)M_PI/2.0f; v < (float)M_PI/2.0f; v += inc){
            Vector3D surface_point = sq.getSurfacePosition(u, v);
            Vector3D surface_normal = sq.getSurfaceNormal(u, v);
            glNormal3f(surface_normal.x, surface_normal.y, surface_normal.z);
            glVertex3f(surface_point.x, surface_point.y, surface_point.z);

            surface_point = sq.getSurfacePosition(u, v+inc);
            surface_normal = sq.getSurfaceNormal(u, v+inc);
            glNormal3f(surface_normal.x, surface_normal.y, surface_normal.z);
            glVertex3f(surface_point.x, surface_point.y, surface_point.z);

            surface_point = sq.getSurfacePosition(u+inc, v+inc);
            surface_normal = sq.getSurfaceNormal(u+inc, v+inc);
            glNormal3f(surface_normal.x, surface_normal.y, surface_normal.z);
            glVertex3f(surface_point.x, surface_point.y, surface_point.z);

            surface_point = sq.getSurfacePosition(u+inc, v);
            surface_normal = sq.getSurfaceNormal(u+inc, v);
            glNormal3f(surface_normal.x, surface_normal.y, surface_normal.z);
            glVertex3f(surface_point.x, surface_point.y, surface_point.z);
        }
    }

    glEnd();
}

void WheeledBoxRenderObject::applyTransform(Transform transform){
    float result[16] = {0.0f};

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

    glMultMatrixf(result);
}
