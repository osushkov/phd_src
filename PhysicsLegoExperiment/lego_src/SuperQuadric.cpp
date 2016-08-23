
#include "SuperQuadric.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>


SuperQuadric::SuperQuadric(double e1, double e2, double A, double B, double C) :
    e1(e1), e2(e2), A(A), B(B), C(C), set(true) {

}

SuperQuadric::SuperQuadric() :
    set(false) {

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
    set = true;

    std::cout << "Loaded SuperQuadric: (" << e1 << "," << e2 << ") " << A << " " << B << " " << C << std::endl;
    return true;
}


bool SuperQuadric::save(std::ostream &out){
    out.write((char*)&e1, sizeof(double));
    out.write((char*)&e2, sizeof(double));

    out.write((char*)&A, sizeof(double));
    out.write((char*)&B, sizeof(double));
    out.write((char*)&C, sizeof(double));

    return out.good();
}


Vector3D SuperQuadric::getSurfacePosition(double u, double v) const {
    Vector3D result;

    result.x = (float)(A*sign(cos(v)*cos(u))*pow(fabs(cos(v)), e1)*pow(fabs(cos(u)), e2));
    result.y = (float)(B*sign(cos(v)*sin(u))*pow(fabs(cos(v)), e1)*pow(fabs(sin(u)), e2));
    result.z = (float)(C*sign(sin(v))*pow(fabs(sin(v)), e1));

    return result;
}


Vector3D SuperQuadric::getSurfaceNormal(double u, double v) const {
    Vector3D result;
    result.x = (float)((1.0/A)*sign(cos(u)*cos(v))*pow(fabs(cos(v)), 2.0-e1)*pow(fabs(cos(u)), 2.0-e2));
    result.y = (float)((1.0/B)*sign(sin(u)*cos(v))*pow(fabs(cos(v)), 2.0-e1)*pow(fabs(sin(u)), 2.0-e2));
    result.z = (float)((1.0/C)*sign(sin(v))*pow(fabs(sin(v)), 2.0-e1));
    result.normalise();

    return result;
}


Vector3D SuperQuadric::getNormalAtPoint(Vector3D point){
    Vector3D result;
    float least_dist = 0.0f;
    float have_result = false;

    const float inc = 0.1f;
    for(float u = -(float)M_PI; u < (float)M_PI; u += inc){
        for(float v = -(float)M_PI/2.0f; v < (float)M_PI/2.0f; v += inc){
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

    for(float u = -(float)M_PI; u < (float)M_PI; u += inc){
        for(float v = -(float)M_PI/2.0f; v < (float)M_PI/2.0f; v += inc){
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
            assert(norm0.x == norm0.x && norm0.y == norm0.y && norm0.z == norm0.z);
            assert(norm1.x == norm1.x && norm1.y == norm1.y && norm1.z == norm1.z);
            assert(fabs(norm0.length()-1) < 0.001f && fabs(norm1.length()-1) < 0.001f);

            result.face_normals.push_back(norm0);
            result.face_normals.push_back(norm1);
        }
    }

    assert(result.face_indices.size() == result.face_normals.size());

    return result;
}

std::vector<Vector3D> SuperQuadric::getSurfacePoints(float inc) const {
    std::vector<Vector3D> result;
    for(float u = -(float)M_PI; u < (float)M_PI; u += inc){
        for(float v = -(float)M_PI/2.0f; v < (float)M_PI/2.0f; v += inc){
            result.push_back(getSurfacePosition(u, v));
        }
    }
    return result;
}

std::vector<Vector3D> SuperQuadric::getPrimaryAxes(void) const {
	std::vector<Vector3D> result;
	if(A > B && A > C){
		result.push_back(Vector3D((float)A, 0.0f, 0.0f));
		if(B > C){
			result.push_back(Vector3D(0.0f, (float)B, 0.0f));
			result.push_back(Vector3D(0.0f, 0.0f, (float)C));
		}
		else{
			result.push_back(Vector3D(0.0f, 0.0f, (float)C));
			result.push_back(Vector3D(0.0f, (float)B, 0.0f));
		}
	}
	else if(B > A && B > C){
		result.push_back(Vector3D(0.0f, (float)B, 0.0f));
		if(A > C){
			result.push_back(Vector3D((float)A, 0.0f, 0.0f));
			result.push_back(Vector3D(0.0f, 0.0f, (float)C));
		}
		else{
			result.push_back(Vector3D(0.0f, 0.0f, (float)C));
			result.push_back(Vector3D((float)A, 0.0f, 0.0f));
		}
	}
	else{
		if(A > B){
			result.push_back(Vector3D(0.0f, 0.0f, (float)C));
			result.push_back(Vector3D((float)A, 0.0f, 0.0f));
			result.push_back(Vector3D(0.0f, (float)B, 0.0f));
		}
		else{
			result.push_back(Vector3D(0.0f, 0.0f, (float)C));
			result.push_back(Vector3D(0.0f, (float)B, 0.0f));
			result.push_back(Vector3D((float)A, 0.0f, 0.0f));
		}
	}
	assert(result.size() == 3);
	return result;
}

double SuperQuadric::findGripSize(Vector3D gripperA, Vector3D gripperB){
    // TODO (this can be done better with a binary search or similar.
    const unsigned iters = 1000;
    Vector3D to_vec = gripperB - gripperA;
    
    bool found_first = false;
    Vector3D contact0(0.0f, 0.0f, 0.0f), contact1(0.0f, 0.0f, 0.0f);

    for(unsigned i = 0; i < iters; i++){
        Vector3D cur_p = gripperA + ((float)i/(float)iters) * to_vec;
        float f = (float)F(cur_p);
        if(!found_first && f <= 1.0f){
            contact0 = cur_p;
            found_first = true;
        }

        if(found_first && f >= 1.0f){
            contact1 = cur_p;
            break;
        }
    }

    std::cout << "Contact points:" << std::endl;
    contact0.print();
    contact1.print();

    Vector3D contact_normal0 = getNormalAtPoint(contact0);
    Vector3D contact_normal1 = getNormalAtPoint(contact1);

    std::cout << "blah:" << (contact0 - contact1).length() << std::endl;
    contact_normal0.print();
    contact_normal1.print();
    return std::max<float>((contact0 - contact1).dotProduct(-1.0f*contact_normal1), (contact1 - contact0).dotProduct(-1.0f*contact_normal0));
}

double SuperQuadric::F(Vector3D p) const {
    double a = pow(fabs(p.x/A), 2.0/e2);
    double b = pow(fabs(p.y/B), 2.0/e2);
    double c = pow(a + b, e2/e1);
    double d = pow(fabs(p.z/C), 2.0/e1);
    return c + d;
}

double SuperQuadric::sign(double x) const {
    if(x < 0.0){ return -1.0; }
    else if(x > 0.0){ return 1.0; }
    else{ return 0.0; }
}

