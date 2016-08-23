
#include "KinectCamera.h"
#include "RemoteKinectCamera.h"


KinectCamera* KinectCamera::getLiveCamera(void){
    return new RemoteKinectCamera("localhost", 9999);
}

KinectCamera* KinectCamera::getPlaybackCamera(std::string path){
    return NULL;
}


static float depth_table[2048];
static bool depth_table_initialised = false;
static void initDepthTable(void){
    const float k1 = 1.1863f;
    const float k2 = 2842.5f;
    const float k3 = 0.1236f;

    for (unsigned i = 0; i < 2048; i++){ 
        const float depth = k3 * tanf(i/k2 + k1); 
        //const float depth = float(1.0 / (double(i) * -0.0030711016 + 3.3309495161));
        depth_table[i] = depth; 
    } 

    depth_table_initialised = true;
}

float KinectCamera::rangeValueToDistance(unsigned range_val){
    if(!depth_table_initialised){
        initDepthTable();
    }

    if(range_val < 2048){
        return depth_table[range_val];
    }    
    else{
        return 0.0f;
    }
}

Vector3D KinectCamera::depthToWorld(unsigned x, unsigned y, float depth_m){
    static const float fx_d = 1.0 / 5.9421434211923247e+02;
    static const float fy_d = 1.0 / 5.9104053696870778e+02;
    static const float cx_d = 3.3930780975300314e+02;
    static const float cy_d = 2.4273913761751615e+02;

    Vector3D result;
    result.x = (x - cx_d) * depth_m * fx_d;
    result.y = (y - cy_d) * depth_m * fy_d;
    result.z = depth_m;

    return result;
}

Vector2D KinectCamera::worldSpaceToRGBPixelCoord(Vector3D coord){

    Matrix3 rot_mat;
    rot_mat(0, 0) = 9.9984628826577793e-01f;
    rot_mat(0, 1) = 1.2635359098409581e-03f;
    rot_mat(0, 2) = -1.7487233004436643e-02f,

    rot_mat(1, 0) = -1.4779096108364480e-03f;
    rot_mat(1, 1) = 9.9992385683542895e-01f;
    rot_mat(1, 2) = -1.2251380107679535e-02f;

    rot_mat(2, 0) = 1.7470421412464927e-02f;
    rot_mat(2, 1) = 1.2275341476520762e-02f;
    rot_mat(2, 2) = 9.9977202419716948e-01f;

    static const Vector3D translation(1.9985242312092553e-02f, -7.4423738761617583e-04f, -1.0916736334336222e-02f);

    static const float fx_rgb = 5.2921508098293293e+02;
    static const float fy_rgb = 5.2556393630057437e+02;
    static const float cx_rgb = 3.2894272028759258e+02;
    static const float cy_rgb = 2.6748068171871557e+02;

    Matrix3 trot_mat;
    trot_mat.isTranspose(rot_mat);
    coord = rot_mat*(coord + translation);
    const float invZ = 1.0f / coord.z;

    Vector2D result;
    result.x = (coord.x * fx_rgb * invZ) + cx_rgb - 5.0f;
    result.y = (coord.y * fy_rgb * invZ) + cy_rgb - 5.0f; // TODO: dodgy hack this is, myeeees.
    return result;
}