/*
 * SuperQuadric.h
 *
 *  Created on: 09/11/2009
 *      Author: osushkov
 */

#ifndef SUPERQUADRIC_H_
#define SUPERQUADRIC_H_

#include <iostream>
#include "Util/Vector3D.h"

class SuperQuadric {
  public:

    SuperQuadric(double e1, double e2, double A, double B, double C);
    SuperQuadric();

    ~SuperQuadric();

    bool load(std::istream &in);
    bool save(std::ostream &out);

    std::vector<float> getParams(void) const;
    void setParams(std::vector<float> params);

    Vector3D getSurfacePosition(double u, double v) const;
    Vector3D getSurfaceNormal(double u, double v) const;

    Vector3D getNormalAtPoint(Vector3D point);

    std::vector<Vector3D> getSurfacePoints(float inc) const;

    double F(Vector3D p) const;
    double distance(Vector3D p) const;
    double distance3(Vector3D p) const;


  private:
    double e1, e2;
    double A, B, C;
    double xt;//, yt;
    bool set;

    std::vector< std::pair<Vector3D,Vector3D> > surface_points; // 2D array of surface points indexed by u and v.
    const unsigned udivs, vdivs;
    const float uinc, vinc;

    double sign(double x) const;
    void getUVLimits(Vector3D p, std::pair<float,float> &ulimits, std::pair<float,float> &vlimits) const;

    Vector3D calculateNormal(Vector3D p0, Vector3D p1, Vector3D p2);
    Vector3D getSurfacePositionNoTaper(double u, double v) const;
};


#endif /* SUPERQUADRIC_H_ */
