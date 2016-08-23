
#ifndef _Geometry_H_
#define _Geometry_H_

#include "Vector2D.h"
#include "Vector3D.h"
#include "Matrix.h"


namespace Geometry {

    bool lineIntersect(Vector3D p1, Vector3D p2, Vector3D p3, Vector3D p4,
                       Vector3D &pa, Vector3D &pb, float &mua, float &mub);

    bool linePlaneIntersect(Vector3D l0, Vector3D l1, Vector3D normal, float d, float &t);

    Vector3D centreOfMass(const std::vector<Vector3D> &points);
    Vector3D findMainAxis(const std::vector<Vector3D> &convex_hull);
    std::vector<Vector3D> axisRotationMatrix(Vector3D axis, float theta);

    Vector2D firstMoment(std::vector<Vector2D> points);
    Vector2D secondMoment(std::vector<Vector2D> points, Vector2D first_moment);

    float linePointDistance(Vector3D line1, Vector3D line2, Vector3D point);
    float linePointDistanceT(Vector3D line1, Vector3D line2, Vector3D point);

    float linePointDistance(Vector2D line1, Vector2D line2, Vector2D point);
    float linePointDistanceT(Vector2D line1, Vector2D line2, Vector2D point);

    Matrix3 findBasis(const std::vector<Vector3D> &triangle);
    Matrix3 matrixFromBasisVectors(const std::vector<Vector3D> &basis_set);

    float minAngleDistance(float angle0, float angle1);
    float triangleArea(std::vector<Vector3D> &triangle);

    std::vector<Vector2D> getLinePixels(Vector2D start, Vector2D end, int inc=1);
};


#endif

