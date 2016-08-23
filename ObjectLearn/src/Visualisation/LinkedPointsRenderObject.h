/*
 * LinkedPointsRenderObject.h
 *
 *  Created on: 16/07/2009
 *      Author: osushkov
 */

#ifndef LINKEDPOINTSRENDEROBJECT_H_
#define LINKEDPOINTSRENDEROBJECT_H_

#include "PointCloudRenderObject.h"
#include "RenderObject.h"



class LinkedPointsRenderObject : public RenderObject {
  public:
    LinkedPointsRenderObject();
    LinkedPointsRenderObject(bool use_texture, Vector3D color);
    ~LinkedPointsRenderObject();

    void addPoints(const std::vector<std::pair<PointCloudPoint,PointCloudPoint> > &new_points);
    void clearPoints(void);

    void setUseTexture(bool var);
    void setColor(Vector3D var);

    void render(void);

  private:
    std::vector<std::pair<PointCloudPoint,PointCloudPoint> > points;

    bool use_texture;
    Vector3D color;


    void renderPoint(const std::pair<PointCloudPoint,PointCloudPoint> &point);
};

#endif /* LINKEDPOINTSRENDEROBJECT_H_ */
