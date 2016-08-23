
#include "SuperQuadric.h"
#include "../Util/Common.h"
#include "../Util/Geometry.h"
#include "../Visualisation/SceneRenderer.h"
#include "../Visualisation/PointCloudRenderObject.h"
#include <cmath>
#include <limits>


float SuperQuadric::shapeDifference(SuperQuadric &sq0, SuperQuadric &sq1){
    float regionA = std::max<float>(sq0.A, sq1.A);
    float regionB = std::max<float>(sq0.B, sq1.B);
    float regionC = std::max<float>(sq0.C, sq1.C);

    Vector3D sq0_offset(0.0f, 0.0f, 0.0f);
    Vector3D sq1_offset(0.0f, 0.0f, 0.0f);

    unsigned num_inside_both = 0;
    unsigned num_inside_one = 0;
    for(unsigned i = 0; i < 500000; i++){
        Vector3D p(rand()/(float)RAND_MAX, rand()/(float)RAND_MAX, rand()/(float)RAND_MAX);
        p.x = (p.x-0.5f)*2.0f*regionA;
        p.y = (p.y-0.5f)*2.0f*regionB;
        p.z = (p.z-0.5f)*2.0f*regionC;

        bool inside_sq0 = sq0.distance2(p - sq0_offset) <= 0.0f;
        bool inside_sq1 = sq1.distance2(p - sq1_offset) <= 0.0f;

        if(inside_sq0 && inside_sq1){
            num_inside_both++;
        }

        if((!inside_sq0 && inside_sq1) || (inside_sq0 && !inside_sq1)){
            num_inside_one++;
        }
    }

    return (float)num_inside_both/(float)(num_inside_both+num_inside_one);
}

void SuperQuadric::shapeDifference2(SuperQuadric &sq0, SuperQuadric &sq1, float &avrg, float &sd, float &max){
    Mesh sq0_mesh = sq0.convertToMesh(M_PI/100.0f);
    Mesh sq1_mesh = sq1.convertToMesh(M_PI/100.0f);

    std::vector<float> dists;
    for(unsigned i = 0; i < sq0_mesh.vertices.size(); i++){
        float closest_dist2 = 0.0f;
        for(unsigned j = 0; j < sq1_mesh.vertices.size(); j++){
            float d2 = (sq0_mesh.vertices[i] - sq1_mesh.vertices[j]).length();
            if(j == 0 || d2 < closest_dist2){
                closest_dist2 = d2;
            }
        }

        dists.push_back(closest_dist2);
    }

    avrg = Common::average(dists);
    sd = Common::standardDeviation(dists);
    max = Common::maximum(dists);
}

double SuperQuadric::F(Vector3D p) const {
    double a = pow(fabs(p.x/A), 2.0/e2);
    double b = pow(fabs(p.y/B), 2.0/e2);
    double c = pow(a + b, e2/e1);
    double d = pow(fabs(p.z/C), 2.0/e1);
    return c + d;
}

double SuperQuadric::distance(Vector3D p) const {
    float zp = (p.z+C)/(2.0*C);
    if(zp < 0.0f){ zp = 0.0f; }
    if(zp > 1.0f){ zp = 1.0f; }
    assert(zp >= 0.0f && zp <= 1.0f);

    //float mod = xt + (1.0f-xt)*zp;//zp*xt + (1.0-zp)*xt;
    float mod = zp*xt + (1.0f-zp);
    p.x = p.x/mod;
    p.y = p.y/mod;

    return p.length() * fabs(1.0 - pow(F(p), -e1/2.0)) * mod;
    //double d = p.length()/pow(F(p), e1/2.0);
    //return d;
}

double SuperQuadric::distance2(Vector3D p) const {
    //return distance(p);
    return findClosestSurfacePointDist2(p);
}

double SuperQuadric::distance3(Vector3D p) const {
    float mod = xt + (1.0-xt)*(p.z+C)/(2.0*C);
    p.x /= mod;
    p.y /= mod;

    return p.length()/pow(F(p), e1/2.0);
/*
    float mod = xt + (1.0-xt)*(p.z+C)/(2.0*C);
    p.x /= mod;
    p.y /= mod;

    double f = F(p);
    assert(f >= 0.0);
    assert(Common::isValidValue(f));
    double fp = pow(f, e1);
    double d = (fp - 1.0)*(fp - 1.0) * mod;

    if(d > 1000000000.0){
        d = 1000000000.0;
    }
    return d;
    */
}

float SuperQuadric::error(std::vector<Vector3D> points){
    float result = 0.0f;
    for(unsigned i = 0; i < surface_points.size(); i++){
        float min_dist2 = 0.0f;
        unsigned best_index = 0;

        for(unsigned j = 0; j < points.size(); j++){
            float d2 = (points[j] - surface_points[i].first).length2();
            if(j == 0 || d2 < min_dist2){
                min_dist2 = d2;
                best_index = j;
            }
        }

        //result += min_dist2;

        Vector3D t = points[best_index] - surface_points[i].first;
        float dp = t.dotProduct(surface_points[i].second);
        if(dp != dp){ continue; }
        result += dp * dp + min_dist2;
        //std::cout << result << " " << dp << " " << min_dist2 << " r" << std::endl;

    }

    //std::cout << "error : " << result << std::endl;
    return result/surface_points.size();
}

SuperQuadric::SuperQuadric(double e1, double e2, double A, double B, double C) :
    e1(e1), e2(e2), A(A), B(B), C(C), xt(1.0), /*yt(1.0),*/ set(true),
    udivs(20), vdivs(20), uinc(2.0f*M_PI/udivs), vinc(M_PI/vdivs) {

    //generateSurfacePoints();
}

SuperQuadric::SuperQuadric(double e1, double e2, double A, double B, double C, double xt, double yt) :
    e1(e1), e2(e2), A(A), B(B), C(C), xt(xt), /*yt(yt),*/ set(true),
    udivs(20), vdivs(20), uinc(2.0f*M_PI/udivs), vinc(M_PI/vdivs) {

    //generateSurfacePoints();
}

SuperQuadric::SuperQuadric(double e1, double e2, double A, double B, double C, double xt, double yt, unsigned mesh_res) :
    e1(e1), e2(e2), A(A), B(B), C(C), xt(xt), /*yt(yt),*/ set(true),
    udivs(mesh_res), vdivs(mesh_res), uinc(2.0f*M_PI/udivs), vinc(M_PI/vdivs) {

    generateSurfacePoints();
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


Mesh SuperQuadric::convertToMesh(float inc) const {
    Mesh result;
/*
    std::cout << "d: " << distance(Vector3D(20,20,20)) << std::endl;

    //for(unsigned i = 0; i < 400; i++){
    while(result.vertices.size() < 400){
        float max_size = (float)std::max<double>(A,std::max<double>(B,C));

        float x = (2.0f*(rand()/(float)RAND_MAX) - 1.0f)*max_size;
        float y = (2.0f*(rand()/(float)RAND_MAX) - 1.0f)*max_size;
        float z = (2.0f*(rand()/(float)RAND_MAX) - 1.0f)*max_size;

        Vector3D pos(x,y,z);
        if(pos.length() > max_size){
            continue;
        }
        pos.normalise();

        Vector3D n = pos;
        pos.scale(max_size);

        float d = distance3(pos);
        pos = d*n;

        result.vertices.push_back(pos);
    }

    return result;
*/
    for(float u = -M_PI; u < M_PI; u += inc){
        for(float v = -M_PI/2.0f; v < M_PI/2.0f; v += inc){
            result.vertices.push_back(getSurfacePosition(u, v));
            result.face_normals.push_back(getSurfaceNormal(u, v));
#if 0
            Vector3D p0 = getSurfacePosition(u, v);
            Vector3D n0 = getSurfaceNormal(u, v);

            Vector3D p1 = getSurfacePosition(u, v+inc);
            Vector3D n1 = getSurfaceNormal(u, v+inc);

            Vector3D p2 = getSurfacePosition(u+inc, v+inc);
            Vector3D n2 = getSurfaceNormal(u+inc, v+inc);

            Vector3D p3 = getSurfacePosition(u+inc, v);
            Vector3D n3 = getSurfaceNormal(u+inc, v);

            std::vector<unsigned> face0, face1;

            face0.push_back(result.vertices.size() + 0);
            face0.push_back(result.vertices.size() + 3);
            face0.push_back(result.vertices.size() + 2);

            face1.push_back(result.vertices.size() + 0);
            face1.push_back(result.vertices.size() + 2);
            face1.push_back(result.vertices.size() + 1);

            result.vertices.push_back(p0);
            result.vertices.push_back(p1);
            result.vertices.push_back(p2);
            result.vertices.push_back(p3);

            result.face_indices.push_back(face0);
            result.face_indices.push_back(face1);

            Vector3D norm0 = n0 + n3 + n2;
            Vector3D norm1 = n0 + n2 + n1;
            norm0.normalise();
            norm1.normalise();
            //assert(norm0.x == norm0.x && norm0.y == norm0.y && norm0.z == norm0.z);
            //assert(norm1.x == norm1.x && norm1.y == norm1.y && norm1.z == norm1.z);
            //assert(fabs(norm0.length()-1) < 0.001f && fabs(norm1.length()-1) < 0.001f);

            result.face_normals.push_back(norm0);
            result.face_normals.push_back(norm1);
#endif
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


double SuperQuadric::sign(double x) const {
    if(x < 0.0){ return -1.0; }
    else if(x > 0.0){ return 1.0; }
    else{ return 0.0; }
}


void SuperQuadric::generateSurfacePoints(void){
    surface_points.clear();//resize(udivs*vdivs);
/*
    for(unsigned vi = 0; vi < vdivs; vi++){
        for(unsigned ui = 0; ui < udivs; ui++){
            float u = -M_PI + ui*uinc;
            float v = -M_PI/2.0f + vi*vinc;

            surface_points[ui + vi*udivs].first = getSurfacePosition(u, v);
            surface_points[ui + vi*udivs].second = getSurfaceNormal(u, v);
        }
    }
*/
    for(unsigned vi = 0; vi < vdivs; vi++){
        for(unsigned ui = 0; ui < udivs; ui++){
    //srand(5);
    //while(surface_points.size() < udivs*vdivs){
        float max_size = (float)std::max<double>(A,std::max<double>(B,C));
/*
        float x = (2.0f*(rand()/(float)RAND_MAX) - 1.0f)*A;
        float y = (2.0f*(rand()/(float)RAND_MAX) - 1.0f)*B;
        float z = (2.0f*(rand()/(float)RAND_MAX) - 1.0f)*C;

        Vector3D pos(x,y,z);
        pos.normalise();*/

            float u = ui*uinc;
            float v = vi*vinc;

            float x = max_size*sin(v)*cos(u);
            float y = max_size*sin(v)*sin(u);
            float z = max_size*cos(v);

            Vector3D pos(x,y,z);
            pos.normalise();

            Vector3D n = pos;
            pos.scale(max_size);

            float d = distance3(pos);
            pos = d*n;

            float xtemp = powf(fabs((pos.x*B)/(pos.y*A)), 2.0f/e2);
            float nx = 1.0f/pos.x * (1 - powf(fabs(pos.z/C), 2.0f/e1)) * (xtemp/(1.0f + xtemp));
            float ny = 1.0f/pos.y * (1 - powf(fabs(pos.z/C), 2.0f/e1)) * (1.0f /(1.0f + xtemp));
            float nz = 1.0f/pos.z * powf(fabs(pos.z/C), 2.0f/e1);

            if(nx != nx){ nx = 0.0f; }
            if(ny != ny){ ny = 0.0f; }
            if(nz != nz){ nz = 0.0f; }
            Vector3D normal(nx, ny, nz);
            normal.normalise();
/*
            normal.print();
            pos.print();
            std::cout << std::endl;
*/
            surface_points.push_back(std::pair<Vector3D,Vector3D>(pos,normal));
        }
    }
/*
    while(surface_points.size() < udivs*vdivs){
        float max_size = (float)std::max<double>(A,std::max<double>(B,C));

        float x = (2.0f*(rand()/(float)RAND_MAX) - 1.0f)*max_size;
        float y = (2.0f*(rand()/(float)RAND_MAX) - 1.0f)*max_size;
        float z = (2.0f*(rand()/(float)RAND_MAX) - 1.0f)*max_size;

        Vector3D pos(x,y,z);
        if(pos.length() > max_size){
            continue;
        }
        pos.normalise();

        Vector3D n = pos;
        pos.scale(max_size);

        float d = distance(pos);
        pos = d*n;

        surface_points.push_back(std::pair<Vector3D,Vector3D>(pos,pos));
        //result.vertices.push_back(pos);
    }
    */

/*
    SceneRenderer::instance().clearObjects();
    PointCloudRenderObject *point_cloud = new PointCloudRenderObject();
    SceneRenderer::instance().addObject(point_cloud);
    point_cloud->setColor(Vector3D(0.0f, 1.0f, 0.0f));

    point_cloud->clearPoints();
    std::vector<PointCloudPoint> all_points;
    for(unsigned i = 0; i < surface_points.size(); i++){
        PointCloudPoint new_point;
        new_point.pos = surface_points[i].first;
        all_points.push_back(new_point);
    }
    point_cloud->addPoints(all_points);

    getchar();
*/

/*
    for(unsigned vi = 0; vi < vdivs; vi++){
        for(unsigned ui = 0; ui < udivs; ui++){
            unsigned prev_vi, prev_ui;
            unsigned next_vi, next_ui;

            if(vi == 0){ prev_vi = vdivs-1; }
            else{ prev_vi = vi-1; }

            if(ui == 0){ prev_ui = udivs-1; }
            else{ prev_ui = ui-1; }

            if(vi == vdivs-1){ next_vi = 0; }
            else{ next_vi = vi+1; }

            if(ui == udivs-1){ next_ui = 0; }
            else{ next_ui = ui+1; }


            Vector3D p0 = surface_points[ui + vi*udivs].first;
            Vector3D p1 = surface_points[ui + prev_vi*udivs].first;
            Vector3D p2 = surface_points[prev_ui + vi*udivs].first;
            Vector3D p3 = surface_points[ui + next_vi*udivs].first;
            Vector3D p4 = surface_points[next_ui + vi*udivs].first;

            std::cout << (p1-p0).length() << std::endl;
            std::cout << (p2-p0).length() << std::endl;
            std::cout << (p3-p0).length() << std::endl;
            std::cout << (p4-p0).length() << std::endl;


            Vector3D n0 = calculateNormal(p0, p1, p2);
            Vector3D n1 = calculateNormal(p0, p2, p3);
            Vector3D n2 = calculateNormal(p0, p3, p4);
            Vector3D n3 = calculateNormal(p0, p4, p1);

            surface_points[ui + vi*udivs].second = n0 + n1 + n2 + n3;
            surface_points[ui + vi*udivs].second.normalise();

            surface_points[ui + vi*udivs].first.print();
            surface_points[ui + vi*udivs].second.print();
            std::cout << std::endl;
            getchar();

            surface_points[ui + vi*udivs].first.print();
            surface_points[ui + vi*udivs].second.print();
            std::cout << std::endl;

        }
    }
    */
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

float SuperQuadric::findClosestSurfacePointDist2(Vector3D p) const {
    std::pair<float,float> ulimits, vlimits;
    getUVLimits(p, ulimits, vlimits);

    std::pair<int,int> uilimits, vilimits;
    uilimits.first = 0;//(int)((ulimits.first + M_PI)/uinc);
    uilimits.second = udivs;//(int)((ulimits.second + M_PI)/uinc);

    vilimits.first = 0;//(int)((vlimits.first + M_PI/2.0f)/vinc);
    vilimits.second = vdivs;//(int)((vlimits.second + M_PI/2.0f)/vinc);

    Vector3D result0;
    Vector3D result1;
    Vector3D result2;
    float min_dist0 = std::numeric_limits<float>::max();
    float min_dist1 = std::numeric_limits<float>::max();
    float min_dist2 = std::numeric_limits<float>::max();
    int best_index0 = 0;
    int best_index1 = 0;
    int best_index2 = 0;

    //for(int vi = vilimits.first; vi < vilimits.second; vi++){
    //    for(int ui = uilimits.first; ui < uilimits.second; ui++){
    for(int index = 0; index < surface_points.size(); index++){
            //int index = ui + vi*udivs;
            float d2 = (p - surface_points[index].first).length2();

            if(d2 < min_dist0){
                result2 = result1;
                min_dist2 = min_dist1;
                best_index2 = best_index1;

                result1 = result0;
                min_dist1 = min_dist0;
                best_index1 = best_index0;

                result0 = surface_points[index].first;
                min_dist0 = d2;
                best_index0 = index;
            }
            else if(d2 < min_dist1){
                result2 = result1;
                min_dist2 = min_dist1;
                best_index2 = best_index1;

                result1 = surface_points[index].first;
                min_dist1 = d2;
                best_index1 = index;
            }
            else if(d2 < min_dist2){
                result2 = surface_points[index].first;
                min_dist2 = d2;
                best_index2 = index;
            }

        //}
    }

/*
    int cur_u = (uilimits.first + uilimits.second)/2;
    int cur_v = (vilimits.first + vilimits.second)/2;
    float cur_d = (p - surface_points[cur_u + cur_u*udivs]).length2();
    Vector3D cur_p = surface_points[cur_u + cur_u*udivs];

    //std::vector<bool> flags((uilimits.second-uilimits.first)*(vilimits.second-vilimits.first), false);

    while(cur_u > uilimits.first && cur_u < uilimits.second-1 &&
          cur_v > vilimits.first && cur_v < vilimits.second-1){
        cmps++;

        std::vector<Vector3D> n_points;
        std::vector<float> n_dist;

        n_points.push_back(surface_points[(cur_u+1) + (cur_v+1)*udivs]);
        n_points.push_back(surface_points[(cur_u-1) + (cur_v+1)*udivs]);
        n_points.push_back(surface_points[(cur_u+1) + (cur_v-1)*udivs]);
        n_points.push_back(surface_points[(cur_u-1) + (cur_v-1)*udivs]);

        unsigned min_index = 0;
        float md = (p-n_points[0]).length2();
        for(unsigned i = 1; i < 4; i++){
            float d = (p-n_points[i]).length2();
            if(d < md){
                md = d;
                min_index = i;
            }
        }

        if(md < cur_d){
            cur_d = md;
            cur_p = n_points[min_index];
            switch(min_index){
            case 0:
                cur_u++;
                cur_v++;
                break;
            case 1:
                cur_u--;
                cur_v++;
                break;
            case 2:
                cur_u++;
                cur_v--;
                break;
            case 3:
                cur_u--;
                cur_v--;
                break;
            }
        }
        else{
            std::cout << "done" << std::endl;
            break;
        }

    }
*/

/*
    Vector3D v0 = result1 - result0;
    Vector3D v1 = result2 - result0;

    std::cout << v0.length() << " " << v1.length() << std::endl;
    std::cout << min_dist0 << " " << min_dist1 << " " << min_dist2 << std::endl;
    v0.normalise();
    v1.normalise();

    Vector3D n = v0.crossProduct(v1);
    n.normalise();

    float a = min_dist0/(min_dist0+min_dist1+min_dist2);
    float b = min_dist1/(min_dist0+min_dist1+min_dist2);
    float c = min_dist2/(min_dist0+min_dist1+min_dist2);
    Vector3D cn = a*result0 + b*result1 + c*result2;
    cn.scale(1.0f/(a+b+c));

    Vector3D n = a*surface_points[best_index0].second + b*surface_points[best_index1].second + c*surface_points[best_index2].second;
    n.normalise();
    */

    Vector3D to_point0 = p - result0;
    float r0 = to_point0.dotProduct(surface_points[best_index0].second);

    Vector3D to_point1 = p - result1;
    float r1 = to_point1.dotProduct(surface_points[best_index1].second);

    Vector3D to_point2 = p - result2;
    float r2 = to_point2.dotProduct(surface_points[best_index2].second);

    return r0+r1+r2;

    //std::cout << sqrtf(min_dist0) << "!" << std::endl;
    return sqrtf(min_dist0);
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
