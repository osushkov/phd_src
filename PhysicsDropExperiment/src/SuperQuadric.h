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
#include "Mesh.h"

class SuperQuadric {
  public:
    SuperQuadric(double e1, double e2, double A, double B, double C);
    SuperQuadric();

    ~SuperQuadric();

    bool load(std::istream &in);
    bool save(std::ostream &out);

    Vector3D getSurfacePosition(double u, double v) const;
    Vector3D getSurfaceNormal(double u, double v) const;

    Vector3D getNormalAtPoint(Vector3D point);

    Mesh convertToMesh(float inc) const;
    std::vector<Vector3D> getSurfacePoints(float inc) const;
    std::vector<Vector3D> getPrimaryAxes(void) const;

    double findGripSize(Vector3D gripperA, Vector3D gripperB);
    double F(Vector3D p) const;

  private:
    double e1, e2;
    double A, B, C;

    bool set;


    double sign(double x) const;

};


#endif /* SUPERQUADRIC_H_ */
