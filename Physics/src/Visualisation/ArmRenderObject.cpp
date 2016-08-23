/*
 * ArmRenderObject.cpp
 *
 *  Created on: 23/07/2010
 *      Author: osushkov
 */

#include "ArmRenderObject.h"
#include "../Util/Common.h"
#include <cassert>

#define _USE_MATH_DEFINES
#include <math.h>


ArmRenderObject::ArmRenderObject(){
	joint_angles = std::vector<float>(6, 0.0f);
	segment_sq = SuperQuadric(0.3, 1.0, 0.5, 0.5, 0.5);
	gripper_sq = SuperQuadric(1.0f, 1.0f, 1.0f, 1.0f, 1.0f);

	worldTransform = Common::armPointToWorldSpaceTransform();
}

ArmRenderObject::~ArmRenderObject(){

}

void ArmRenderObject::setUseTexture(bool var){
	use_texture = var;
}

void ArmRenderObject::setColor(Vector3D var){
	color = var;
}

void ArmRenderObject::render(void){
	std::pair<Vector3D,Vector3D> gripper_pos = afk.getGripperPosition(joint_angles);

	assert(joint_angles.size() == 6);
	glEnable(GL_NORMALIZE);
	float segment_width = 5.0f;

	glPushMatrix();

	applyWorldTransform();

	glPushMatrix();
	glTranslatef(gripper_pos.first.x/10.0f, gripper_pos.first.y/10.0f, gripper_pos.first.z/10.0f);
	renderSegment(1.0f, 1.0f, Vector3D(1.0f, 1.0f, 1.0f), gripper_sq, false);
	glPopMatrix();

	glPushMatrix();
	glTranslatef(gripper_pos.second.x/10.0f, gripper_pos.second.y/10.0f, gripper_pos.second.z/10.0f);
	renderSegment(1.0f, 1.0f, Vector3D(1.0f, 1.0f, 1.0f), gripper_sq, false);
	glPopMatrix();



	glTranslatef(0.0f, 0.0f, 15.5f);
	glRotatef(joint_angles[0], 0.0f, 0.0f, 1.0f);
	renderSegment(12.5f, segment_width, Vector3D(1.0f, 1.0f, 1.0f), segment_sq);

	glTranslatef(0.0f, 0.0f, 12.5f);
	glRotatef(joint_angles[1], 0.0f, 1.0f, 0.0f);
	renderSegment(21.0f, segment_width, Vector3D(1.0f, 1.0f, 1.0f), segment_sq);

	glTranslatef(0.0f, 0.0f, 21.0f);
	glRotatef(joint_angles[2], 0.0f, 1.0f, 0.0f);
	glPushMatrix();
	glTranslatef(-7.5f, 0.0f, -5.0f);
	renderSegment(14.0f, segment_width, Vector3D(1.0f, 1.0f, 1.0f), segment_sq);
	glPopMatrix();

	glTranslatef(-7.5f, 0.0f, 9.0f);
	glRotatef(joint_angles[3], 0.0f, 0.0f, 1.0f);
	renderSegment(12.0f, segment_width, Vector3D(1.0f, 1.0f, 1.0f), segment_sq);

	glTranslatef(0.0f, 0.0f, 12.0f);
	glRotatef(joint_angles[4], 0.0f, 1.0f, 0.0f);
	renderSegment(7.0f, segment_width, Vector3D(1.0f, 1.0f, 1.0f), segment_sq);

	glTranslatef(0.0f, 0.0f, 7.0f);
	glRotatef(joint_angles[5], 0.0f, 0.0f, 1.0f);
	renderSegment(18.0f, segment_width, Vector3D(1.0f, 1.0f, 1.0f), segment_sq);

	glPopMatrix();


}

void ArmRenderObject::setJointAngles(std::vector<float> angles){
	joint_angles = angles;
}

void ArmRenderObject::moveTo(std::vector<float> dst_angles, float duration){
	const unsigned frames = 100;
	unsigned ms_pause = duration*1000000/frames;

	std::vector<float> start_joint_angles = joint_angles;
	for(unsigned i = 0; i < frames; i++){
		for(unsigned j = 0; j < joint_angles.size(); j++){
			joint_angles[j] = (float)i/(float)frames * dst_angles[j] + (1.0f - (float)i/(float)frames) * start_joint_angles[j];
		}

#ifdef _WIN32
        Sleep(ms_pause/1000);
#else
		usleep(ms_pause);
#endif
	}

}

void ArmRenderObject::renderSegment(float length, float width, Vector3D color, SuperQuadric sq, bool render_axis){
    glPushMatrix();


    glDisable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);
    glTranslatef(0.0f, 0.0f, length);

    float b_length = 5.0f;
    if(render_axis){
		glBegin(GL_LINES);

		glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
		glVertex3f(0.0f, 0.0f, 0.0f);
		glVertex3f(b_length, 0.0f, 0.0f);

		glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
		glVertex3f(0.0f, 0.0f, 0.0f);
		glVertex3f(0.0f, b_length, 0.0f);

		glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
		glVertex3f(0.0f, 0.0f, 0.0f);
		glVertex3f(0.0f, 0.0f, b_length);

		glEnd();
    }

    glPopMatrix();

    glPushMatrix();
    glTranslatef(0.0f, 0.0f, length/2.0f);
    glScalef(width, width, length);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    glBegin(GL_QUADS);

    const float inc = 0.1f;
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
    glPopMatrix();
}

void ArmRenderObject::applyWorldTransform(void){
	float result[16] = {0.0f};

    result[0] = worldTransform(0, 0);
    result[1] = worldTransform(1, 0);
    result[2] = worldTransform(2, 0);
    result[3] = worldTransform(3, 0);

    result[4] = worldTransform(0, 1);
    result[5] = worldTransform(1, 1);
    result[6] = worldTransform(2, 1);
    result[7] = worldTransform(3, 1);

    result[8] = worldTransform(0, 2);
    result[9] = worldTransform(1, 2);
    result[10] = worldTransform(2, 2);
    result[11] = worldTransform(3, 2);

    result[12] = worldTransform(0, 3);
    result[13] = worldTransform(1, 3);
    result[14] = worldTransform(2, 3);
    result[15] = worldTransform(3, 3);

    glMultMatrixf(result);
}
