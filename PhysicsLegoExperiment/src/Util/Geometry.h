
#ifndef _Geometry_H_
#define _Geometry_H_

#include "Common.h"
#include "Vector2D.h"
#include "Vector3D.h"
#include "Matrix.h"

namespace Geometry {

    Matrix3 getMatrixFromTo(Vector3D from, Vector3D to);

    bool lineIntersect(Vector3D p1, Vector3D p2, Vector3D p3, Vector3D p4,
                       Vector3D &pa, Vector3D &pb, float &mua, float &mub);

    Vector3D centreOfMass(const std::vector<Vector3D> &points);
    Vector3D findMainAxis(const std::vector<Vector3D> &convex_hull);
    Matrix3 axisRotationMatrix(Vector3D axis, float theta);

    Vector2D firstMoment(std::vector<Vector2D> points);
    Vector2D secondMoment(std::vector<Vector2D> points, Vector2D first_moment);

    float linePointDistance(Vector3D line1, Vector3D line2, Vector3D point);
    float linePointDistanceT(Vector3D line1, Vector3D line2, Vector3D point);

    Matrix3 findBasis(const std::vector<Vector3D> &triangle);
    Matrix3 matrixFromBasisVectors(const std::vector<Vector3D> &basis_set);

    float minAngleDistance(float angle0, float angle1);
    float triangleArea(std::vector<Vector3D> &triangle);


    std::vector<float> makeQuaternion(float a, float b, float c, float d);
    float quaternionAngleDiff(std::vector<float> vec0, std::vector<float> vec1);
    float vectorDotProduct(std::vector<float> vec0, std::vector<float> vec1);

    std::vector<Vector3D> pointsOnSphere(const unsigned n);

    std::pair<Vector3D, Vector3D> perpedicularBasis(Vector3D vec);
};


#endif

