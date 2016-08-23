
#include "Common.h"
#include "Neck.h"
#include "../SIFT/sift.h"
#include "../StereoFeatureCorrelation.h"

#include <cmath>


static Vector3D getNeckCoordinates(Vector3D cam, float panAngle, float tiltAngle, float rollAngle);
static Vector3D getGripperCoordinates(Vector3D neck);
static Vector3D getArmCoordinates(Vector3D grip);


bool Common::lineIntersect(Vector3D p1, Vector3D p2, Vector3D p3, Vector3D p4, 
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


void Common::extractFeatures(IplImage* img, std::vector<feature> &dump_buffer){
    
    struct feature* features;
    int n = SIFT::sift_features(img, &features);

    dump_buffer.clear();
    for(int i = 0; i < n; i++){
        dump_buffer.push_back(features[i]);
    }

    free(features); 
}

Vector3D Common::getProjectedCameraPoint(float pix_x, float pix_y){
    Vector3D result;
    result.x = 0.0f; result.y = 0.0f; result.z = 0.0f;

    //pix_x += 13.0f;
    //pix_y -= 16.0f;

    float theta = 40.0f/2.0f;
    float phi = 30.0/2.0f;

    float width = 320.0f;
    float height = 240.0f;

    float lx = -tan(theta*M_PI/180.0f);
    float rx = tan(theta*M_PI/180.0f);

    float ty = tan(phi*M_PI/180.0f);
    float by = -tan(phi*M_PI/180.0f);

    float px = (float)pix_x/width * (rx - lx) + lx;
    float py = (float)pix_y/height * (by - ty) + ty;
    float pz = 1.0f;

    float length = sqrtf(px*px + py*py + pz*pz);
    result.x = px/length * 200.0f;
    result.y = py/length * 200.0f;
    result.z = pz/length * 200.0f;
    
    return result;
}

float Common::featureDist(StereoFeature &feature1, StereoFeature &feature2){
    return sqrtf(descr_dist_sq(&(feature1.feature_left), &(feature2.feature_left))) +
           sqrtf(descr_dist_sq(&(feature1.feature_right), &(feature2.feature_right)));
}

float Common::vectorDist(Vector3D var1, Vector3D var2){
    return sqrtf((var1.x-var2.x)*(var1.x-var2.x) + (var1.y-var2.y)*(var1.y-var2.y) + (var1.z-var2.z)*(var1.z-var2.z));
}



Vector3D Common::cameraPointToArmSpace(Vector3D point){
    Vector3D neck_pos = getNeckCoordinates(point, 430, 140, 440);
    Vector3D gripper_pos = getGripperCoordinates(neck_pos);
    Vector3D arm_pos = getArmCoordinates(gripper_pos);

    return gripper_pos;
    return arm_pos;
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



