
#include "Common.h"
#include "Geometry.h"
#include "../Features/SIFT/sift.h"
#include "../Features/StereoFeatureCorrelation.h"
#include "../Settings.h"

#define _USE_MATH_DEFINES
#include <math.h>

#define LEFT_CAMERA_X_OFFSET (-6.0f)
#define RIGHT_CAMERA_X_OFFSET (6.0f)


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

void Common::extractFeatures(IplImage* img, std::vector<feature> &dump_buffer){

    struct feature* features;
    int n = SIFT::sift_features(img, &features);

    dump_buffer.clear();
    for(int i = 0; i < n; i++){
        dump_buffer.push_back(features[i]);
    }

    free(features);
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

    //std::cout << "theta, phi: " << theta << " " << phi << std::endl;

    float width = (float)Settings::instance().getIntValue("camera", "res_width");
    float height = (float)Settings::instance().getIntValue("camera", "res_height");

    float lx = -tanf(theta*(float)M_PI/180.0f);
    float rx = tanf(theta*(float)M_PI/180.0f);

    float ty = tanf(phi*(float)M_PI/180.0f);
    float by = -tanf(phi*(float)M_PI/180.0f);

    float px = (float)pix_x/width * (rx - lx) + lx;
    float py = (float)pix_y/height * (by - ty) + ty;
    float pz = 1.0f;

    float length = sqrtf(px*px + py*py + pz*pz);
    result.x = px/length * 300.0f; // TODO: 300 is 3metres and considered to be "infinity"
    result.y = py/length * 300.0f;
    result.z = pz/length * 300.0f;

    return result;
}

float Common::featureDist(const StereoFeature &feature1, const StereoFeature &feature2){
    return sqrtf(descr_dist_sq(&(feature1.feature_left), &(feature2.feature_left))) +
           sqrtf(descr_dist_sq(&(feature1.feature_right), &(feature2.feature_right)))/2.0f;
}

float Common::featureDist2(const StereoFeature &feature1, const StereoFeature &feature2){
    return descr_dist_sq(&(feature1.feature_left), &(feature2.feature_left)) +
           descr_dist_sq(&(feature1.feature_right), &(feature2.feature_right));
}

std::vector<StereoFeature> Common::findNearestFeatures(const std::vector<StereoFeature> &pool,
                                                       const StereoFeature &target){
    float nearest_dist = FLT_MAX, nearest_dist2 = FLT_MAX;
    StereoFeature nearest_feature, nearest_feature2;

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

    std::vector<StereoFeature> result;
    result.push_back(nearest_feature);
    result.push_back(nearest_feature2);
    return result;
}

float Common::gaussianNoise(float mean, float sd){
    // Taken from GSL Library Gaussian random distribution.
    float x, y, r2;

    do{
        /* choose x,y in uniform square (-1,-1) to (+1,+1) */
        x = -1 + 2 * (float)rand()/(float)RAND_MAX;
        y = -1 + 2 * (float)rand()/(float)RAND_MAX;

        /* see if it is in the unit circle */
        r2 = x * x + y * y;
    } while (r2 > 1.0 || r2 == 0);

    /* Box-Muller transform */
    return mean + sd * y * sqrtf (-2.0f * logf (r2) / r2);
}

float Common::normalDistribution(float mean, float sd, float x){
    return exp(-(x-mean)*(x-mean)/(2.0f*sd*sd));
}

void Common::writeFeature(std::ostream &out, StereoFeature &feature){
    write_feature(out, feature.feature_left);
    write_feature(out, feature.feature_right);
    out.write((char*)&(feature.position), sizeof(Vector3D));
}


void Common::readFeature(std::istream &in, StereoFeature &feature){
    read_feature(in, feature.feature_left);
    read_feature(in, feature.feature_right);
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

void Common::formProbabilityDistribution(std::vector<float> &vec){
	float sum = 0.0f;
	for(unsigned i = 0; i < vec.size(); i++){
	    sum += vec[i];
	}

	for(unsigned i = 0; i < vec.size(); i++){
		vec[i] /= sum;
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

void Common::printVector(std::vector<float> vec){
    for(unsigned i = 0; i < vec.size(); i++){
        if(i > 0){
            std::cout << " ";
        }
        std::cout << vec[i];
    }
    std::cout << std::endl;
}
