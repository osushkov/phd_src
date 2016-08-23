/*
 * ArmRenderObject.h
 *
 *  Created on: 23/07/2010
 *      Author: osushkov
 */

#ifndef ARMRENDEROBJECT_H_
#define ARMRENDEROBJECT_H_

#include "RenderObject.h"
#include "../Util/Vector3D.h"
#include "../SuperQuadric.h"
#include "../Arm/ArmForwardKinematics.h"

#include <GL/glu.h>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif


class ArmRenderObject : public RenderObject {
  public:
	ArmRenderObject();
    ~ArmRenderObject();

    void setUseTexture(bool var);
    void setColor(Vector3D var);

    void render(void);

    void setJointAngles(std::vector<float> angles);
    void moveTo(std::vector<float> angles, float duration);

  private:
    Matrix4 worldTransform;
    std::vector<float> joint_angles;

    bool use_texture;
    Vector3D color;
    SuperQuadric segment_sq, gripper_sq;

    ArmForwardKinematics afk;


    void renderSegment(float length, float width, Vector3D color, SuperQuadric sq, bool render_axis=true);
    void applyWorldTransform(void);
};

#endif /* ARMRENDEROBJECT_H_ */
