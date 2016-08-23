/*
 * SuperQuadric.h
 *
 *  Created on: 09/11/2009
 *      Author: osushkov
 */

#ifndef SUPERQUADRIC_H_
#define SUPERQUADRIC_H_

#include <iostream>
#include "../Util/Vector3D.h"
#include "../Mesh.h"

class SuperQuadric {
  public:

    float F(Vector3D p) const;
    float distance(Vector3D p) const;
    float distance2(Vector3D p) const;
    float distance3(Vector3D p) const;

    float error(std::vector<Vector3D> points);

    SuperQuadric(float e1, float e2, float A, float B, float C);
    SuperQuadric(float e1, float e2, float A, float B, float C, float xt, float yt);
    SuperQuadric(float e1, float e2, float A, float B, float C, float xt, float yt, unsigned mesh_res);
    SuperQuadric();

    ~SuperQuadric();

    bool load(std::istream &in);
    bool save(std::ostream &out);

    std::vector<float> getParams(void) const;
    void setParams(std::vector<float> params);

    Vector3D getSurfacePosition(float u, float v) const;
    Vector3D getSurfaceNormal(float u, float v) const;

    Vector3D getNormalAtPoint(Vector3D point);

    Mesh convertToMesh(float inc) const;
    std::vector<Vector3D> getSurfacePoints(float inc) const;

private:
    float e1, e2;
    float A, B, C;
    float xt;//, yt;
    bool set;

    std::vector< std::pair<Vector3D,Vector3D> > surface_points; // 2D array of surface points indexed by u and v.
    const unsigned udivs, vdivs;
    const float uinc, vinc;

    float sign(float x) const;
    void generateSurfacePoints(void);
    void getUVLimits(Vector3D p, std::pair<float,float> &ulimits, std::pair<float,float> &vlimits) const;
    float findClosestSurfacePointDist2(Vector3D p) const ;

    Vector3D calculateNormal(Vector3D p0, Vector3D p1, Vector3D p2);
    Vector3D getSurfacePositionNoTaper(float u, float v) const;
};


#endif /* SUPERQUADRIC_H_ */
