
#include "Geometry.h"
#include <cassert>

#define _USE_MATH_DEFINES
#include <math.h>

Matrix3 Geometry::getMatrixFromTo(Vector3D from, Vector3D to){
    from.normalise();
    to.normalise();

    // TODO: handle reflected vectors too.
    if(fabs(from.dotProduct(to) - 1.0f) < 0.001f){
    	Matrix3 result;
    	result.identity();
    	return result;
    }

    Vector3D perp = from.crossProduct(to);
    perp.normalise();

    float theta = acosf(from.dotProduct(to));
    return axisRotationMatrix(perp, theta);
}

bool Geometry::lineIntersect(Vector3D p1, Vector3D p2, Vector3D p3, Vector3D p4,
                             Vector3D &pa, Vector3D &pb, float &mua, float &mub){

    Vector3D p13,p43,p21;
    float d1343,d4321,d1321,d4343,d2121;
    float numer,denom;
    const float EPS = 0.001f;

    p13.x = p1.x - p3.x;
    p13.y = p1.y - p3.y;
    p13.z = p1.z - p3.z;
    p43.x = p4.x - p3.x;
    p43.y = p4.y - p3.y;
    p43.z = p4.z - p3.z;
    if (fabs(p43.x) < EPS && fabs(p43.y) < EPS && fabs(p43.z) < EPS){
        return false;
    }

    p21.x = p2.x - p1.x;
    p21.y = p2.y - p1.y;
    p21.z = p2.z - p1.z;
    if (fabs(p21.x) < EPS && fabs(p21.y) < EPS && fabs(p21.z) < EPS){
        return false;
    }

    d1343 = p13.x * p43.x + p13.y * p43.y + p13.z * p43.z;
    d4321 = p43.x * p21.x + p43.y * p21.y + p43.z * p21.z;
    d1321 = p13.x * p21.x + p13.y * p21.y + p13.z * p21.z;
    d4343 = p43.x * p43.x + p43.y * p43.y + p43.z * p43.z;
    d2121 = p21.x * p21.x + p21.y * p21.y + p21.z * p21.z;

    denom = d2121 * d4343 - d4321 * d4321;
    if(fabs(denom) < EPS){
      return false;
    }

    if(fabs(d4343) < EPS){
      return false;
    }

    numer = d1343 * d4321 - d1321 * d4343;

    mua = numer / denom;
    mub = (d1343 + d4321 * (mua)) / d4343;

    pa.x = p1.x + mua * p21.x;
    pa.y = p1.y + mua * p21.y;
    pa.z = p1.z + mua * p21.z;
    pb.x = p3.x + mub * p43.x;
    pb.y = p3.y + mub * p43.y;
    pb.z = p3.z + mub * p43.z;

    return true;
}

Vector3D Geometry::centreOfMass(const std::vector<Vector3D> &points){
    Vector3D cm(0.0f, 0.0f, 0.0f);
    for(unsigned i = 0; i < points.size(); i++){
        cm.add(points[i]);
    }

    cm.scale(1.0f/points.size());
    return cm;
}

Vector3D Geometry::findMainAxis(const std::vector<Vector3D> &convex_hull){
    Vector3D result;
    float largest_dist = 0.0f;

    for(unsigned i = 0; i < convex_hull.size(); i++){
        for(unsigned j = i+1; j < convex_hull.size(); j++){
            float dist = (convex_hull[i] - convex_hull[j]).length();
            if(dist > largest_dist){
                largest_dist = dist;
                result = (convex_hull[i] - convex_hull[j]);
            }
        }
    }

    return result;
}

Matrix3 Geometry::axisRotationMatrix(Vector3D axis, float theta){
    Matrix3 result;

    float c = cosf(theta);
    float s = sinf(theta);
    float t = 1.0f - cosf(theta);

    result(0,0) = t*axis.x*axis.x + c;
    result(1,0) = t*axis.x*axis.y +s*axis.z;
    result(2,0) = t*axis.x*axis.z -s*axis.y;

    result(0,1) = t*axis.x*axis.y - s*axis.z;
    result(1,1) = t*axis.y*axis.y + c;
    result(2,1) = t*axis.y*axis.z + s*axis.x;

    result(0,2) = t*axis.x*axis.z +s*axis.y;
    result(1,2) = t*axis.y*axis.z -s*axis.x;
    result(2,2) = t*axis.z*axis.z + c;

    /*
    Vector3D col1(t*axis.x*axis.x + c, t*axis.x*axis.y +s*axis.z, t*axis.x*axis.z -s*axis.y);
    Vector3D col2(t*axis.x*axis.y - s*axis.z, t*axis.y*axis.y + c, t*axis.y*axis.z + s*axis.x);
    Vector3D col3(t*axis.x*axis.z +s*axis.y, t*axis.y*axis.z -s*axis.x, t*axis.z*axis.z + c);

    result.push_back(col1);
    result.push_back(col2);
    result.push_back(col3);
    */

    return result;
}

Vector2D Geometry::firstMoment(std::vector<Vector2D> points){
    Vector2D result(0.0f, 0.0f);

    for(unsigned i = 0; i < points.size(); i++){
        result.x += points[i].x;
        result.y += points[i].y;
    }

    result.scale(1.0f/points.size());
    return result;
}

Vector2D Geometry::secondMoment(std::vector<Vector2D> points, Vector2D first_moment){
    float a = 0.0f, b = 0.0f, c = 0.0f;

    for(unsigned i = 0; i < points.size(); i++){
        float x = points[i].x - first_moment.x;
        float y = points[i].y - first_moment.y;
        a += x*x;
        b += x*y;
        c += y*y;
    }

    b *= 2.0f;

    float theta = atan2(b, a-c+sqrtf((a-c)*(a-c)+b*b));
    return Vector2D(cos(theta), sin(theta));
}

float Geometry::linePointDistance(Vector3D line1, Vector3D line2, Vector3D point){
    Vector3D t1 = point - line1;
    Vector3D t2 = point - line2;
    return (t1.crossProduct(t2)).length()/(line2 - line1).length();
}

float Geometry::linePointDistanceT(Vector3D line1, Vector3D line2, Vector3D point){
    Vector3D t1 = line1 - point;
    Vector3D t2 = line2 - line1;
    return t1.dotProduct(t2)/(line2 - line1).length2();
}

Matrix3 Geometry::findBasis(const std::vector<Vector3D> &triangle){
    assert(triangle.size() == 3);

    Vector3D v0 = triangle[1] - triangle[0];
    Vector3D v1 = triangle[2] - triangle[0];

    v0.normalise();
    v1.normalise();

    Vector3D norm = v0.crossProduct(v1);
    norm.normalise();

    Vector3D binorm = norm.crossProduct(v0);
    binorm.normalise();

    Matrix3 result;
    result(0, 0) = v0.x;
    result(1, 0) = v0.y;
    result(2, 0) = v0.z;

    result(0, 1) = norm.x;
    result(1, 1) = norm.y;
    result(2, 1) = norm.z;

    result(0, 2) = binorm.x;
    result(1, 2) = binorm.y;
    result(2, 2) = binorm.z;

    return result;
}

Matrix3 Geometry::matrixFromBasisVectors(const std::vector<Vector3D> &basis_set){
    assert(basis_set.size() == 3);
    Matrix3 result;

    result(0, 0) = basis_set[0].x;
    result(1, 0) = basis_set[0].y;
    result(2, 0) = basis_set[0].z;

    result(0, 1) = basis_set[1].x;
    result(1, 1) = basis_set[1].y;
    result(2, 1) = basis_set[1].z;

    result(0, 2) = basis_set[2].x;
    result(1, 2) = basis_set[2].y;
    result(2, 2) = basis_set[2].z;

    return result;
}

float Geometry::minAngleDistance(float angle0, float angle1){
    while(angle0 < 0.0f){ angle0 += 2.0f*(float)M_PI; }
    while(angle1 < 0.0f){ angle1 += 2.0f*(float)M_PI; }

    while(angle0 >= 2.0f*M_PI){ angle0 -= 2.0f*(float)M_PI; }
    while(angle1 >= 2.0f*M_PI){ angle1 -= 2.0f*(float)M_PI; }

    float min_angle = std::min<float>(angle0, angle1);
    float max_angle = std::max<float>(angle0, angle1);

    return std::min<float>(max_angle - min_angle, min_angle + 2.0f*(float)M_PI - max_angle);
}

float Geometry::triangleArea(std::vector<Vector3D> &triangle){
    float a = (triangle[0]-triangle[1]).length();
    float b = (triangle[1]-triangle[2]).length();
    float c = (triangle[2]-triangle[0]).length();

    float s = (a+b+c)/2.0f; // semi-perimeter
    return sqrtf(s*(s-a)*(s-b)*(s-c));
}

std::vector<float> Geometry::makeQuaternion(float a, float b, float c, float d){
    std::vector<float> result;
    result.push_back(a);
    result.push_back(b);
    result.push_back(c);
    result.push_back(d);
    Common::normaliseVector(result);
    return result;
}

float Geometry::quaternionAngleDiff(std::vector<float> vec0, std::vector<float> vec1){
    float dp = vectorDotProduct(vec0, vec1);
    return acosf(dp);
}

float Geometry::vectorDotProduct(std::vector<float> vec0, std::vector<float> vec1){
    assert(vec0.size() == vec1.size());

    float result = 0.0f;
    for(unsigned i = 0; i < vec0.size(); i++){
        result += vec0.at(i)*vec1.at(i);
    }
    return result;
}

std::vector<Vector3D> Geometry::pointsOnSphere(const unsigned n) {
    double dl = M_PI*(3.0 - sqrt(5.0));
    double dz = 2.0/(double)n;
    double l = 0.0;
    double z = 1.0 - dz/2.0;
   
    std::vector<Vector3D> result;
    for(unsigned i = 0; i < n; i++){
        double r = sqrt(1.0 - z*z);

        Vector3D p(cos(l)*r, sin(l)*r, z);
        p.normalise();
        result.push_back(p);
        
        z -= dz;
        l += dl;
    }

    return result;
}