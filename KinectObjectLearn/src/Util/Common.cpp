#include "Common.h"
#include "Neck.h"
#include "Geometry.h"
#include "../Features/SIFT/sift.h"
#include "../Features/SIFTFeature3D.h"
#include "../KinectCamera/KinectCamera.h"
#include "../Settings.h"


#define _USE_MATH_DEFINES
#include <math.h>
#include <limits>

static Vector3D getNeckCoordinates(Vector3D cam, float panAngle, float tiltAngle, float rollAngle);
static Vector3D getGripperCoordinates(Vector3D neck);
static Vector3D getArmCoordinates(Vector3D grip);

void Common::sleep(unsigned ms){
#ifdef _WIN32
    Sleep(ms);
#else
    if(ms > 1000){
        sleep(ms/1000);
        usleep((ms%1000)*1000)
    }
    else{
        usleep(ms*1000);
    }
#endif
}

bool Common::isPointInWorkspace(Vector3D wpoint){
    return (wpoint.z > 17.0f && wpoint.y > 10.0f && wpoint.y < 70.0f && 
            wpoint.x < 40.0f && wpoint.x > -40.0f);
}

bool Common::isValidValue(float val){
    return val == val && val != std::numeric_limits<float>::infinity();
}

bool Common::isValidValue(double val){
    return val == val && val != std::numeric_limits<double>::infinity();
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

void Common::extractFeature(KinectCamera::CorrelatedImage &frame, std::vector<SIFTFeature3D> &out_features){
    IplImage* img_rgb = imageFromCorrelatedFrame(frame);

    std::vector<feature> sift_buffer;
    extractFeatures(img_rgb, sift_buffer);
    cvReleaseImage(&img_rgb);

    out_features.clear();
    for(unsigned i = 0; i < sift_buffer.size(); i++){
        // TODO: maybe do some kind of depth pixel interpolation here to take 
        // advantage of the sub-pixel accuracy of SIFT pixel location.
        int x = (int)sift_buffer[i].x;
        int y = (int)sift_buffer[i].y;

        if(x >= 0 && x < (int)frame.width && y >= 0 && y < (int)frame.height && 
           frame.depth_pixels[x + y*frame.width].have_pos){
            SIFTFeature3D new_feature;
            new_feature.sift_feature = sift_buffer[i];
            new_feature.position = frame.depth_pixels[x + y*frame.width].pos;
            if(new_feature.position.length() > 0.1f){
                out_features.push_back(new_feature);
            }
        }
    }
}

IplImage* Common::imageFromCorrelatedFrame(KinectCamera::CorrelatedImage &frame){
    IplImage* img_rgb = cvCreateImage(cvSize(frame.width, frame.height), IPL_DEPTH_8U, 3);
    for(unsigned i = 0; i < frame.depth_pixels.size(); i++){
        img_rgb->imageData[i*3 + 0] = (char)frame.depth_pixels[i].color.z;
        img_rgb->imageData[i*3 + 1] = (char)frame.depth_pixels[i].color.y;
        img_rgb->imageData[i*3 + 2] = (char)frame.depth_pixels[i].color.x;
    }
    return img_rgb;
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


float Common::featureDist(const SIFTFeature3D &feature1, const SIFTFeature3D &feature2){
    return sqrtf(descr_dist_sq(&(feature1.sift_feature), &(feature2.sift_feature)));
}

float Common::featureDist2(const SIFTFeature3D &feature1, const SIFTFeature3D &feature2){
    return descr_dist_sq(&(feature1.sift_feature), &(feature2.sift_feature));
}

std::vector<SIFTFeature3D> Common::findNearestFeatures(const std::vector<SIFTFeature3D> &pool,
                                                       const SIFTFeature3D &target){
    float nearest_dist = FLT_MAX, nearest_dist2 = FLT_MAX;
    SIFTFeature3D nearest_feature, nearest_feature2;

    for(unsigned i = 0; i < pool.size(); i++){
        float d = featureDist(target, pool[i]);
        if(d < nearest_dist){
            nearest_dist2 = nearest_dist;
            nearest_feature2 = nearest_feature;

            nearest_dist = d;
            nearest_feature = pool[i];
        }
        else if(d < nearest_dist2){
            nearest_dist2 = d;
            nearest_feature2 = pool[i];
        }
    }

    std::vector<SIFTFeature3D> result;
    result.push_back(nearest_feature);
    result.push_back(nearest_feature2);
    return result;
}

Vector3D Common::cameraPointToWorldSpace(Vector3D point, float pan, float tilt){
    point.scale(100.0f);
    point.z = -point.z;
    point.x = -point.x;
    point.x -= 4.0f;
    point.z += 1.0f;

    std::vector<Vector3D> rot_mat ;
    Vector3D result = point;
    result = result + Vector3D(0.0f, 8.0f, -3.0f);

    rot_mat = Geometry::axisRotationMatrix(Vector3D(1.0f, 0.0f, 0.0f), tilt*(float)M_PI/180.0f);
    result.matrixMultLeft(rot_mat);
    result = result + Vector3D(0.0f, 0.0f, 3.5f);

    rot_mat = Geometry::axisRotationMatrix(Vector3D(0.0f, 1.0f, 0.0f), pan*(float)M_PI/180.0f);
    result.matrixMultLeft(rot_mat);

    result = result + Vector3D(0.0f, 0.0f, 10.0f);
    result = result + Vector3D(0.0f, 8.5f, 0.0f);
    result = result + Vector3D(0.0f, 0.0f, 82.0f);
    result = result + Vector3D(0.0f, -8.0f, 0.0f);

    return result;
}

Transform Common::cameraToWorldSpaceTransform(float pan, float tilt){
    std::vector<Vector3D> basis;
    basis.push_back(Vector3D(1.0f, 0.0f, 0.0f));
    basis.push_back(Vector3D(0.0f, 1.0f, 0.0f));
    basis.push_back(Vector3D(0.0f, 0.0f, 1.0f));
    basis.push_back(Vector3D(0.0f, 0.0f, 0.0f));

    Transform result;
    result.mat.identity();
    result.shift = cameraPointToWorldSpace(basis[3], pan, tilt);
    for(unsigned i = 0; i < 3; i++){
        Vector3D t = cameraPointToWorldSpace(basis[i], pan, tilt);
        result.mat(0, i) = t.x - result.shift.x;
        result.mat(1, i) = t.y - result.shift.y;
        result.mat(2, i) = t.z - result.shift.z;
    }

    return result;
}

Vector3D Common::armPointToWorldSpace(Vector3D point){
    Vector3D result;

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

float Common::curveDistance(const std::vector<Vector3D> &curve1,
                            const std::vector<Vector3D> &curve2){
    std::vector<Vector3D> transformed_curve1;
    std::vector<Vector3D> transformed_curve2;
    unsigned limit = std::min(curve1.size(), curve2.size());

    // Move both curves such that they both start at the origin.
    for(unsigned i = 0; i < limit; i++){
        transformed_curve1.push_back(curve1.at(i));
        transformed_curve2.push_back(curve2.at(i));
    }

    // Take the sum of the distances between each point pair along the curves.
    float result = 0.0f;
    unsigned num = 0;
    for(unsigned i = 0; i < limit-1; i++){
        Vector3D v0 = transformed_curve1.at(i+1) - transformed_curve1.at(i);
        Vector3D v1 = transformed_curve2.at(i+1) - transformed_curve2.at(i);

        if(v0.length() > 0.1f && v1.length() > 0.1f){
            v0.normalise();
            v1.normalise();

            result += acosf(v0.dotProduct(v1))*180.0f/M_PI;//(transformed_curve1.at(i) - transformed_curve2.at(i)).length();
            num++;
        }
    }


    if(num != 0){
        result /= (float)num;
    }

    return result;
}

float Common::curveDistance2(const std::vector<Vector3D> &curve1,
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

void Common::writeFeature(std::ostream &out, const SIFTFeature3D &feature){
    write_feature(out, feature.sift_feature);
    out.write((char*)&(feature.position), sizeof(Vector3D));
}

void Common::readFeature(std::istream &in, SIFTFeature3D &feature){
    read_feature(in, feature.sift_feature);
    in.read((char*)&(feature.position), sizeof(Vector3D));
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

float Common::maximum(const std::vector<float> &data){
    float result = 0.0f;
    for(unsigned i = 0; i < data.size(); i++){
        if(i == 0 || data[i] > result){
            result = data[i];
        }
    }
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

IplImage* Common::convertToGray32(const IplImage* img){
    IplImage* gray8, *gray32;

    gray8 = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
    gray32 = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F, 1);

    if(img->nChannels == 1){
      gray8 = (IplImage*)cvClone(img);
    }
    else {
      cvCvtColor(img, gray8, CV_BGR2GRAY);
    }
    cvConvertScale(gray8, gray32, 1.0/255.0, 0);
    cvReleaseImage(&gray8);

    return gray32;
}

std::vector<Vector3D> Common::basisVectorsFromTriangle(std::vector<Vector3D> &triangle){
    assert(triangle.size() == 3);
    std::vector<Vector3D> result;

    Vector3D side0 = triangle[1] - triangle[0];
    Vector3D side1 = triangle[2] - triangle[0];

    side0.normalise();
    side1.normalise();

    result.push_back(side0.crossProduct(side1));
    result[0].normalise();

    result.push_back(side0.crossProduct(result[0]));
    result[1].normalise();

    result.push_back(side0);

    return result;

}

Vector3D getNeckCoordinates(Vector3D cam, float panAngle, float tiltAngle, float rollAngle) {
    Vector3D result;

	//Adjust coordinates so that we are at the base of the neck
	cam.z += CAM_2_NECK_Z;
	cam.y += CAM_2_NECK_Y;

    float H = sqrt(cam.y*cam.y + cam.z*cam.z);

	float camAngleG = 2.0f*(float)M_PI*(tiltAngle - TILT_CENTER)/DXL_360;	//get cam tilt angle in radians
	float objAngleL = atan2f(cam.y, cam.z);						//object angle from the cam center
	float objAngleG = camAngleG + objAngleL;
	float neckYZ = H * cosf(objAngleG);
	result.y = (H * sinf(objAngleG));
	result.y += NECK_2_NECKBASE;

    //Remember, PAN increases to the left, but we want a negative degree to the left.
	H = sqrt(cam.x*cam.x + neckYZ*neckYZ);
	camAngleG = 2.0f*(float)M_PI*(PAN_CENTER - panAngle)/DXL_360;		//get cam pan angle in radians
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




