
#include "Common.h"
#include "Neck.h"
#include "Geometry.h"
#include "../Settings.h"

#include <cmath>

#define LEFT_CAMERA_X_OFFSET (-6.0f)
#define RIGHT_CAMERA_X_OFFSET (6.0f)


static Vector3D getNeckCoordinates(Vector3D cam, float panAngle, float tiltAngle, float rollAngle);
static Vector3D getGripperCoordinates(Vector3D neck);
static Vector3D getArmCoordinates(Vector3D grip);


Vector3D Common::stereoProject(Vector2D left_coord, Vector2D right_coord){
    float avrg_y_coord = (left_coord.y + right_coord.y)*0.5f;
    left_coord.y = right_coord.y = avrg_y_coord;

    Vector3D far_left = Common::getProjectedCameraPoint(left_coord.x, left_coord.y);
    Vector3D far_right = Common::getProjectedCameraPoint(right_coord.x, right_coord.y);

    Vector3D origin_left, origin_right;
    origin_left.y = origin_left.z = origin_right.y = origin_right.z = 0.0f;
    origin_left.x = LEFT_CAMERA_X_OFFSET;
    origin_right.x = RIGHT_CAMERA_X_OFFSET;

    far_left.x += LEFT_CAMERA_X_OFFSET;
    far_right.x += RIGHT_CAMERA_X_OFFSET;

    Vector3D r1, r2;
    r1.x = r2.x = r1.y = r2.y = r1.z = r2.z = 0.0f;
    float d1 = 0.0f, d2 = 0.0f;
    Geometry::lineIntersect(origin_left, far_left, origin_right, far_right, r1, r2, d1, d2);

    Vector3D avrg;
    avrg.x = (r1.x + r2.x)/2.0f;
    avrg.y = (r1.y + r2.y)/2.0f;
    avrg.z = (r1.z + r2.z)/2.0f;

    return avrg;
}

IplImage* Common::imageFromBuffer(const std::vector<unsigned char> &buffer,
                                  unsigned width, unsigned height, int Bpp){
    assert(Bpp == 1 || Bpp == 3);
    IplImage* img = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, Bpp);

    for(unsigned y = 0; y < height; y++){
        for(unsigned x = 0; x < width; x++){
            unsigned index = (x + y*width)*Bpp;
            assert(index < buffer.size());

            if(Bpp == 1){
                setPixel(img, x, y, buffer.at(index));
            }
            else{
                setPixel(img, x, y, buffer.at(index), buffer.at(index+1), buffer.at(index+2));
            }
        }
    }

    return img;
}


void Common::getPixel(IplImage* img, unsigned x, unsigned y, double &val){
    CvScalar s;
    s = cvGet2D(img, y, x);
    val = s.val[0];
}


void Common::getPixel(IplImage* img, unsigned x, unsigned y, double &b, double &g, double &r){
    CvScalar s;
    s = cvGet2D(img, y, x);
    b = s.val[0];
    g = s.val[1];
    r = s.val[2];
}


void Common::setPixel(IplImage* img, unsigned x, unsigned y, double val){
    CvScalar s;
    s.val[0] = val;
    cvSet2D(img, y, x, s);
}


void Common::setPixel(IplImage* img, unsigned x, unsigned y, double b, double g, double r){
    CvScalar s;
    s.val[0] = b;
    s.val[1] = g;
    s.val[2] = r;
    cvSet2D(img, y, x, s);
}


Vector3D Common::getProjectedCameraPoint(float pix_x, float pix_y){
    Vector3D result;
    result.x = 0.0f; result.y = 0.0f; result.z = 0.0f;

    float theta = Settings::instance().getFloatValue("camera", "xfov")/2.0f;
    float phi = Settings::instance().getFloatValue("camera", "yfov")/2.0f;

    float width = Settings::instance().getIntValue("camera", "res_width");
    float height = Settings::instance().getIntValue("camera", "res_height");

    float lx = -tan(theta*M_PI/180.0f);
    float rx = tan(theta*M_PI/180.0f);

    float ty = tan(phi*M_PI/180.0f);
    float by = -tan(phi*M_PI/180.0f);

    float px = (float)pix_x/width * (rx - lx) + lx;
    float py = 1.0f;
    float pz = (float)pix_y/height * (by - ty) + ty;

    float length = sqrtf(px*px + py*py + pz*pz);
    result.x = px/length * 300.0f; // TODO: 300 is 3metres and considered to be "infinity"
    result.y = py/length * 300.0f;
    result.z = pz/length * 300.0f;

    return result;
}

Vector3D Common::cameraPointToArmSpace(Vector3D point){
    Vector3D neck_pos = getNeckCoordinates(point, 430, 140, 440);
    Vector3D gripper_pos = getGripperCoordinates(neck_pos);
    Vector3D arm_pos = getArmCoordinates(gripper_pos);

    return gripper_pos;
    return arm_pos;
}

Vector3D Common::cameraPointToWorldSpace(Vector3D point, float pan, float tilt, float roll){
    std::vector<Vector3D> rot_mat ;
    Vector3D result = point;
    result = result + Vector3D(0.0f, 9.8f, 0.0f);

    rot_mat = Geometry::axisRotationMatrix(Vector3D(0.0f, 0.0f, 1.0f), roll*M_PI/180.0f);
    result.matrixMultLeft(rot_mat);
    result = result + Vector3D(0.0f, 2.6, 3.5f);

    rot_mat = Geometry::axisRotationMatrix(Vector3D(1.0f, 0.0f, 0.0f), tilt*M_PI/180.0f);
    result.matrixMultLeft(rot_mat);
    result = result + Vector3D(0.0f, 0.0f, 3.5f);

    rot_mat = Geometry::axisRotationMatrix(Vector3D(0.0f, 1.0f, 0.0f), pan*M_PI/180.0f);
    result.matrixMultLeft(rot_mat);

    result = result + Vector3D(0.0f, 0.0f, 10.0f);
    result = result + Vector3D(0.0f, 8.5f, 0.0f);
    result = result + Vector3D(0.0f, 0.0f, 82.0f);
    result = result + Vector3D(0.0f, -8.0f, 0.0f);

    return result;
}

Vector3D Common::armPointToWorldSpace(Vector3D point){
    Vector3D result;

    point.x /= 10.0f;
    point.y /= 10.0f;
    point.z /= 10.0f;

    point.z += SPINE_2_ARMBASE;
    point.y = -point.y;

    float H = sqrt(point.x*point.x + point.z*point.z);
	float objAngleG = atan2(point.x, point.z); //sign
	float objAngleL = objAngleG - TURN_2_LEFTARM;	//The angle from the spine turned towards the left arm
	point.z = H * cos(objAngleL);
	point.x = H * sin(objAngleL);

    result.x = point.x;
    result.y = point.z - WORLD_Y_TO_SPINE;
	result.z = WORLD_Z_TO_SPINE + point.y;

    return result;
}

Vector3D Common::worldPointToArmSpace(Vector3D point){
    Vector3D result;

	//translation to the origin of arm base

	result.x = point.x;
	result.y = WORLD_Z_TO_SPINE - point.z;
	result.z = point.y + WORLD_Y_TO_SPINE;


	float H = sqrt(result.x*result.x + result.z*result.z);
	float objAngleG = atan2(result.x, result.z);
	float objAngleL = TURN_2_LEFTARM + objAngleG;	//The angle from the spine turned towards the left arm
	result.z = H * cos(objAngleL);
	result.x = H * sin(objAngleL);

	result.z -= SPINE_2_ARMBASE;

	result.x *= 10.0f;	//Centimeters to Millimeters format required by the arm.
	result.y *= 10.0f;
	result.z *= 10.0f;

    return result;
}


Vector3D Common::cameraPointToWorldSpace(Vector3D point){
    Vector3D neck_coords = getNeckCoordinates(point, 430, 140, 440);
    neck_coords.z += NECKBASE_2_SPINE;

    Vector3D result;
    result.x = neck_coords.x;
    result.y = neck_coords.z + TOP_SPINE_TO_WORLD_Y;
	result.z = neck_coords.y + TOP_SPINE_TO_WORLD_Z;

    return result;
}

float Common::curveDistance(const std::vector<Vector3D> &curve1,
                            const std::vector<Vector3D> &curve2){
    std::vector<Vector3D> transformed_curve1;
    std::vector<Vector3D> transformed_curve2;
    unsigned limit = std::min(curve1.size(), curve2.size());

    // Move both curves such that they both start at the origin.
    for(unsigned i = 0; i < limit; i++){
        transformed_curve1.push_back(curve1.at(i) - curve1.at(0));
        transformed_curve2.push_back(curve2.at(i) - curve2.at(0));
    }

    // Take the sum of the distances between each point pair along the curves.
    float result = 0.0f;
    for(unsigned i = 0; i < limit; i++){
        result += (transformed_curve1.at(i) - transformed_curve2.at(i)).length();
    }

    if(limit != 0){
        result /= (float)limit;
    }

    return result;
}

float Common::average(const std::vector<float> &data){
    float result = 0.0f;
    for(unsigned i = 0; i < data.size(); i++){
        result += data[i];
    }
    result /= data.size();
    return result;
}

float Common::standardDeviation(const std::vector<float> &data){
    float average = Common::average(data);
    float result = 0.0f;
    for(unsigned i = 0; i < data.size(); i++){
        result += (data[i]-average)*(data[i]-average);
    }
    result = sqrtf(result/data.size());
    return result;
}

Matrix3 Common::quaternionsToMatrix(std::vector<float> quaternions){
    assert(quaternions.size() == 4);

    float q0 = quaternions[0];
    float q1 = quaternions[1];
    float q2 = quaternions[2];
    float q3 = quaternions[3];

    Matrix3 result;

    result(0, 0) = q0*q0 + q1*q1 - q2*q2 - q3*q3;
    result(0, 1) = 2.0f*(q1*q2 + q0*q3);
    result(0, 2) = 2.0f*(q1*q3 - q0*q2);

    result(1, 0) = 2.0f*(q1*q2 - q0*q3);
    result(1, 1) = q0*q0 - q1*q1 + q2*q2 - q3*q3;
    result(1, 2) = 2.0f*(q2*q3 + q0*q1);

    result(2, 0) = 2.0f*(q1*q3 + q0*q2);
    result(2, 1) = 2.0f*(q2*q3 - q0*q1);
    result(2, 2) = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    return result;
}

std::vector<float> Common::matrixToQuaternions(Matrix3 &mat){
    std::vector<float> result;

    unsigned ld = 0;
    if(mat(1,1) > mat(ld,ld)){
        ld = 1;
    }
    if(mat(2,2) > mat(ld,ld)){
        ld = 2;
    }

    //std::cout << mat(0,0)+mat(1,1)+mat(2,2) << " trace" << std::endl;

    unsigned vi = (ld + 1)%3;
    unsigned wi = (ld + 2)%3;

    //std::cout << mat(ld,ld)+mat(1,1)+mat(2,2)

    float r = sqrtf(1.0f + mat(ld,ld) - mat(vi,vi) - mat(wi,wi));
    result.push_back((mat(wi,vi) - mat(vi,wi))/(2.0f*r));

    result.push_back(0.0f);
    result.push_back(0.0f);
    result.push_back(0.0f);

    result[ld+1] = r/2.0f;
    result[vi+1] = (mat(ld,vi) + mat(vi,ld)/(2.0f*r));
    result[wi+1] = (mat(ld,wi) + mat(wi,ld)/(2.0f*r));

    return result;
}

void Common::normaliseVector(std::vector<float> &vec){
    float length = 0.0f;
    for(unsigned i = 0; i < vec.size(); i++){
        length += vec[i]*vec[i];
    }
    length = 1.0f/sqrtf(length);
    for(unsigned i = 0; i < vec.size(); i++){
        vec[i] *= length;
    }
}

Vector3D getNeckCoordinates(Vector3D cam, float panAngle, float tiltAngle, float rollAngle) {
    Vector3D result;

	//Adjust coordinates so that we are at the base of the neck
	cam.z += CAM_2_NECK_Z;
	cam.y += CAM_2_NECK_Y;

    float H = sqrt(cam.y*cam.y + cam.z*cam.z);

	float camAngleG = 2.0f*M_PI*(tiltAngle - TILT_CENTER)/DXL_360;	//get cam tilt angle in radians
	float objAngleL = atan2f(cam.y, cam.z);						//object angle from the cam center
	float objAngleG = camAngleG + objAngleL;
	float neckYZ = H * cosf(objAngleG);
	result.y = (H * sinf(objAngleG));
	result.y += NECK_2_NECKBASE;

    //Remember, PAN increases to the left, but we want a negative degree to the left.
	H = sqrt(cam.x*cam.x + neckYZ*neckYZ);
	camAngleG = 2.0f*M_PI*(PAN_CENTER - panAngle)/DXL_360;		//get cam pan angle in radians
	objAngleL = atan2f(cam.x, neckYZ);							//object angle from the cam center
	objAngleG = camAngleG + objAngleL;
	result.x = H * sinf(objAngleG);
	result.z = H * cosf(objAngleG);

    return result;
}

Vector3D getGripperCoordinates(Vector3D neck){
    Vector3D result;

	neck.z += NECKBASE_2_SPINE;
	float H = sqrtf(neck.x*neck.x + neck.z*neck.z);
	float objAngleG = atan2f(neck.x, neck.z);
	float objAngleL = TURN_2_LEFTARM - objAngleG;			//The angle from the spine turned towards the left arm

	result.z = H * cosf(objAngleL);
	result.x = H * sinf(objAngleL);
	result.y = -1.0f*(neck.y + ALONG_THE_SPINE);
	result.z -= SPINE_2_ARMBASE;

	result.x *= 10.0f;  //Centimeters to Millimeters format required by the arm.
	result.y *= 10.0f;
	result.z *= 10.0f;

    return result;
}

Vector3D getArmCoordinates(Vector3D grip){
    Vector3D result;

	result.x = grip.x - GRIPPER_X;
	result.y = grip.y - GRIPPER_Y;
	result.z = grip.z;

    return result;
}

// should be for ttes2 (204, 374, 380)



