
#include "SuperQuadric.h"
#include "Util/Common.h"
#include "Util/Geometry.h"
#include <cmath>
#include <limits>

#define _USE_MATH_DEFINES
#include <math.h>

SuperQuadric::SuperQuadric(double e1, double e2, double A, double B, double C) :
    e1(e1), e2(e2), A(A), B(B), C(C), xt(1.0), /*yt(1.0),*/ set(true),
    udivs(20), vdivs(20), uinc(2.0f*M_PI/udivs), vinc(M_PI/vdivs) {

    //generateSurfacePoints();
}

SuperQuadric::SuperQuadric() :
    set(false), udivs(20), vdivs(20), uinc(2.0f*M_PI/udivs), vinc(M_PI/vdivs) {

}

SuperQuadric::~SuperQuadric(){

}


bool SuperQuadric::load(std::istream &in) {
    if(!in.good() || in.bad() || in.eof()){
        return false;
    }

    in.read((char*)&e1, sizeof(double));
    in.read((char*)&e2, sizeof(double));

    in.read((char*)&A, sizeof(double));
    in.read((char*)&B, sizeof(double));
    in.read((char*)&C, sizeof(double));

    in.read((char*)&xt, sizeof(double));
    set = true;

    //A = 10.0f;
    std::cout << e1 << " " << e2 << " " << A << " " << B << " " << C << " " << xt << std::endl;
/*
    e1 = 0.152f;
    e2 = 0.01f;
    A = 5.29f;
    B = 4.021f;
    C = 2.07f;
*/
    return true;
}


bool SuperQuadric::save(std::ostream &out){
    out.write((char*)&e1, sizeof(double));
    out.write((char*)&e2, sizeof(double));

    out.write((char*)&A, sizeof(double));
    out.write((char*)&B, sizeof(double));
    out.write((char*)&C, sizeof(double));

    out.write((char*)&xt, sizeof(double));

    return out.good();
}

std::vector<float> SuperQuadric::getParams(void) const {
    std::vector<float> result;

    result.push_back(e1);
    result.push_back(e2);

    result.push_back(A);
    result.push_back(B);
    result.push_back(C);
    assert(false);

    return result;
}

void SuperQuadric::setParams(std::vector<float> params){
    assert(params.size() == 5);

    e1 = params[0];
    e2 = params[1];

    A = params[2];
    B = params[3];
    C = params[4];
    assert(false);
}

Vector3D SuperQuadric::getSurfacePosition(double u, double v) const {
    Vector3D result;

    result.x = (float)(A*sign(cos(v)*cos(u))*pow(fabs(cos(v)), e1)*pow(fabs(cos(u)), e2));
    result.y = (float)(B*sign(cos(v)*sin(u))*pow(fabs(cos(v)), e1)*pow(fabs(sin(u)), e2));
    result.z = (float)(C*sign(sin(v))*pow(fabs(sin(v)), e1));


    float zp = (result.z+C)/(2.0*C);
    if(zp < 0.0f){ zp = 0.0f; }
    if(zp > 1.0f){ zp = 1.0f; }
    assert(zp >= 0.0f && zp <= 1.0f);

    //float mod = xt + (1.0f-xt)*zp;//zp*xt + (1.0-zp)*xt;
    float mod = zp*xt + (1.0f-zp);
    result.x *= mod;
    result.y *= mod;

    //result.x *= (float)(xt + (1.0-xt)*(result.z+C)/(2.0*C));
    //result.y *= (float)(xt + (1.0-xt)*(result.z+C)/(2.0*C));

    //result.print();
    return result;
}


Vector3D SuperQuadric::getSurfaceNormal(double u, double v) const {
    Vector3D result;
    result.x = (float)((1.0/A)*sign(cos(u)*cos(v))*pow(fabs(cos(v)), 2.0-e1)*pow(fabs(cos(u)), 2.0-e2));
    result.y = (float)((1.0/B)*sign(sin(u)*cos(v))*pow(fabs(cos(v)), 2.0-e1)*pow(fabs(sin(u)), 2.0-e2));
    result.z = (float)((1.0/C)*sign(sin(v))*pow(fabs(sin(v)), 2.0-e1));
    result.normalise();
/*
    Vector3D spoint = getSurfacePosition(u, v);
    Vector3D spoint_nt = getSurfacePositionNoTaper(u, v);

    Vector3D stem_point = spoint;
    stem_point.x = stem_point.y = 0.0f;

    Vector3D from_stem = spoint - stem_point;
    from_stem.normalise();

    Vector3D axis_rotation = Vector3D(0.0f, 0.0f, 1.0f).crossProduct(from_stem);
    axis_rotation.normalise();


    float x = (spoint_nt - spoint).length();
    float l = spoint.z - C;

    float theta = atan2f(x, l);
    std::vector<Vector3D> bvecs = Geometry::axisRotationMatrix(axis_rotation, theta);

    Matrix3 rot_mat = Geometry::matrixFromBasisVectors(bvecs);
    //result = rot_mat*result;
    result.normalise();
*/

    return result;
}


Vector3D SuperQuadric::getNormalAtPoint(Vector3D point){
    Vector3D result;
    float least_dist = 0.0f;
    float have_result = false;

    const float inc = 0.1f;
    for(float u = -M_PI; u < M_PI; u += inc){
        for(float v = -M_PI/2.0f; v < M_PI/2.0f; v += inc){
            Vector3D surface_point = getSurfacePosition(u, v);
            float d = (point-surface_point).length2();
            if(!have_result || d < least_dist){
                have_result = true;
                least_dist = d;
                result = getSurfaceNormal(u, v);
            }
        }
    }

    return result;
}

std::vector<Vector3D> SuperQuadric::getSurfacePoints(float inc) const {
    std::vector<Vector3D> result;
    for(float u = -M_PI; u < M_PI; u += inc){
        for(float v = -M_PI/2.0f; v < M_PI/2.0f; v += inc){
            result.push_back(getSurfacePosition(u, v));
        }
    }
    return result;
}

double SuperQuadric::F(Vector3D p) const {
    double a = pow(fabs(p.x/A), 2.0/e2);
    double b = pow(fabs(p.y/B), 2.0/e2);
    double c = pow(a + b, e2/e1);
    double d = pow(fabs(p.z/C), 2.0/e1);
    return c + d;
}

double SuperQuadric::distance(Vector3D p) const {
    return p.length() * fabs(1.0 - pow(F(p), -e1/2.0));
    //double d = p.length()/pow(F(p), e1/2.0);
    //return d;
}

double SuperQuadric::distance3(Vector3D p) const {
    float mod = xt + (1.0-xt)*(p.z+C)/(2.0*C);
    p.x /= mod;
    p.y /= mod;

    return p.length()/pow(F(p), e1/2.0);
}

double SuperQuadric::sign(double x) const {
    if(x < 0.0){ return -1.0; }
    else if(x > 0.0){ return 1.0; }
    else{ return 0.0; }
}

void SuperQuadric::getUVLimits(Vector3D p, std::pair<float,float> &ulimits, std::pair<float,float> &vlimits) const {
    if(p.z >= 0.0f){
        vlimits.first = 0.0f;
        vlimits.second = M_PI/2.0f;
    }
    else{
        vlimits.first = -M_PI/2.0f;
        vlimits.second = 0.0f;
    }

    if(p.x >= 0.0f && p.y >= 0.0f){
        ulimits.first = 0.0f;
        ulimits.second = M_PI/2.0f;
    }
    else if(p.x < 0 && p.y >= 0.0f){
        ulimits.first = M_PI/2.0f;
        ulimits.second = M_PI;
    }
    else if(p.x >= 0.0f && p.y < 0.0f){
        ulimits.first = -M_PI/2.0f;
        ulimits.second = 0.0f;
    }
    else if(p.x < 0 && p.y < 0.0f){
        ulimits.first = -M_PI;
        ulimits.second = -M_PI/2.0f;
    }
    else{
        p.print();
        assert(false);
    }
}


Vector3D SuperQuadric::calculateNormal(Vector3D p0, Vector3D p1, Vector3D p2){
    Vector3D e0 = p1 - p0;
    Vector3D e1 = p2 - p0;
    e0.normalise();
    e1.normalise();

    Vector3D normal = e0.crossProduct(e1);
    normal.normalise();
    return normal;
}

Vector3D SuperQuadric::getSurfacePositionNoTaper(double u, double v) const {
    Vector3D result;

    result.x = (float)(A*sign(cos(v)*cos(u))*pow(fabs(cos(v)), e1)*pow(fabs(cos(u)), e2));
    result.y = (float)(B*sign(cos(v)*sin(u))*pow(fabs(cos(v)), e1)*pow(fabs(sin(u)), e2));
    result.z = (float)(C*sign(sin(v))*pow(fabs(sin(v)), e1));

    return result;
}
