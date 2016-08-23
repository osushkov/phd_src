/*
 * PointCloudRenderObject.h
 *
 *  Created on: 12/07/2009
 *      Author: osushkov
 */

#ifndef POINTCLOUDRENDEROBJECT_H_
#define POINTCLOUDRENDEROBJECT_H_

#include "RenderObject.h"
#include "../Util/Vector3D.h"
#include "../Util/Vector2D.h"
#include "../Util/Semaphore.h"

#include <vector>


struct PointCloudPoint {
    Vector3D pos;
    Vector2D img_pos;
    unsigned texture_id;
    Vector3D color;
};

class PointCloudRenderObject : public RenderObject {
  public:
    PointCloudRenderObject();
    PointCloudRenderObject(bool use_texture, Vector3D color);
    ~PointCloudRenderObject();

    void addPoints(const std::vector<PointCloudPoint> &new_points);
    void clearPoints(void);

    void setUseTexture(bool var);
    void setColor(Vector3D var);

    void render(void);

  private:
    std::vector<PointCloudPoint> points;

    bool use_texture;
    Vector3D color;
    Util::Semaphore lock;

    void renderPoint(const PointCloudPoint &point);
};


#endif /* POINTCLOUDRENDEROBJECT_H_ */
