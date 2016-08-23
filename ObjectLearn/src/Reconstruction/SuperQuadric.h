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
    static float shapeDifference(SuperQuadric &sq0, SuperQuadric &sq1);
    static void shapeDifference2(SuperQuadric &sq0, SuperQuadric &sq1, float &avrg, float &sd, float &max);

    double F(Vector3D p) const;
    double distance(Vector3D p) const;
    double distance2(Vector3D p) const;
    double distance3(Vector3D p) const;

    float error(std::vector<Vector3D> points);

    SuperQuadric(double e1, double e2, double A, double B, double C);
    SuperQuadric(double e1, double e2, double A, double B, double C, double xt, double yt);
    SuperQuadric(double e1, double e2, double A, double B, double C, double xt, double yt, unsigned mesh_res);
    SuperQuadric();

    ~SuperQuadric();

    bool load(std::istream &in);
    bool save(std::ostream &out);

    std::vector<float> getParams(void) const;
    void setParams(std::vector<float> params);

    Vector3D getSurfacePosition(double u, double v) const;
    Vector3D getSurfaceNormal(double u, double v) const;

    Vector3D getNormalAtPoint(Vector3D point);

    Mesh convertToMesh(float inc) const;
    std::vector<Vector3D> getSurfacePoints(float inc) const;

private:
    double e1, e2;
    double A, B, C;
    double xt;//, yt;
    bool set;

    std::vector< std::pair<Vector3D,Vector3D> > surface_points; // 2D array of surface points indexed by u and v.
    const unsigned udivs, vdivs;
    const float uinc, vinc;

    double sign(double x) const;
    void generateSurfacePoints(void);
    void getUVLimits(Vector3D p, std::pair<float,float> &ulimits, std::pair<float,float> &vlimits) const;
    float findClosestSurfacePointDist2(Vector3D p) const ;

    Vector3D calculateNormal(Vector3D p0, Vector3D p1, Vector3D p2);
    Vector3D getSurfacePositionNoTaper(double u, double v) const;
};


#endif /* SUPERQUADRIC_H_ */
