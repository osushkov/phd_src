
#include "KinectCamera.h"
#include "RemoteKinectCamera.h"
#include "PlaybackKinectCamera.h"


KinectCamera* KinectCamera::getLiveCamera(void){
    return new RemoteKinectCamera("localhost", 9999);
}

KinectCamera* KinectCamera::getPlaybackCamera(std::string path){
    return new PlaybackKinectCamera(path);
}

KinectCamera::CorrelatedImage KinectCamera::getCorrelatedImage(KinectCamera *cam, bool blocking) {
    unsigned rgb_width = cam->getRGBImageWidth();
    unsigned rgb_height = cam->getRGBImageHeight();
    unsigned depth_width = cam->getDepthImageWidth();
    unsigned depth_height = cam->getDepthImageHeight();

    std::vector<unsigned char> lrgb_buffer;
    std::vector<unsigned> ldepth_buffer;
    cam->getImage(lrgb_buffer, ldepth_buffer, blocking);

    std::vector<std::pair<Vector3D,bool> > pixel_positions(rgb_width*rgb_height, std::pair<Vector3D,bool>(Vector3D(0.0f, 0.0f, 0.0f), false));

    for(unsigned y = 0; y < depth_height; y++){
        for(unsigned x = 0; x < depth_width; x++){
            float zdepth = KinectCamera::rangeValueToDistance(ldepth_buffer[x + y*depth_width]);
            if(zdepth < 0.1f){
                continue;
            }

            Vector3D world_pos = KinectCamera::depthToWorld(x, y, zdepth);

            Vector2D point_rgb_xy = KinectCamera::worldSpaceToRGBPixelCoord(world_pos);
            if(point_rgb_xy.x < 0.0f || point_rgb_xy.x >= rgb_width ||
                point_rgb_xy.y < 0.0f || point_rgb_xy.y >= rgb_height){
                    continue;
            }

            unsigned px = (unsigned)point_rgb_xy.x;
            unsigned py = (unsigned)point_rgb_xy.y;
            pixel_positions[px + py*rgb_width] = std::pair<Vector3D,bool>(world_pos, true);
        }
    }

    const int x_window = 3;
    const int y_window = 3;
    for(int y = 0; y < rgb_height; y++){
        for(int x = 0; x < rgb_width; x++){
            if(pixel_positions[x + y*rgb_width].second){ continue; }

            int x_ref_left = x;
            int x_ref_right = x;
            for(int i = 1; i < x_window; i++){
                if(x_ref_right == x && 
                   x+i < rgb_width && 
                   pixel_positions[x+i + y*rgb_width].second){
                    x_ref_right = x+i;
                }

                if(x_ref_left == x &&
                   x >= i &&
                   pixel_positions[x-i + y*rgb_width].second){
                    x_ref_left = x-i;
                }
            }

            if(x_ref_left != x && x_ref_right != x){
                Vector3D interp_pos = pixel_positions[x_ref_left + y*rgb_width].first + 
                    (x-x_ref_left)/(x_ref_right-x_ref_left) * 
                    (pixel_positions[x_ref_right + y*rgb_width].first - pixel_positions[x_ref_left + y*rgb_width].first);

                pixel_positions[x + y*rgb_width] = std::pair<Vector3D,bool>(interp_pos, true);
                continue;
            }


            int y_ref_up = y;
            int y_ref_down = y;
            for(int i = 1; i < y_window; i++){
                if(y_ref_down == y && 
                    y+i < rgb_height && 
                    pixel_positions[x + (y+i)*rgb_width].second){

                        y_ref_down = y+i;
                }

                if(y_ref_up == y &&
                    y >= i &&
                    pixel_positions[x + (y-i)*rgb_width].second){

                        y_ref_up = y-i;
                }
            }

            if(y_ref_up != y && y_ref_down != y){
                Vector3D interp_pos = pixel_positions[x + y_ref_up*rgb_width].first + 
                    (y-y_ref_up)/(y_ref_down-y_ref_up) * 
                    (pixel_positions[x + y_ref_down*rgb_width].first - pixel_positions[x + y_ref_up*rgb_width].first);

                pixel_positions[x + y*rgb_width] = std::pair<Vector3D,bool>(interp_pos, true);
            }
        }
    }


    CorrelatedImage result;
    result.width = rgb_width;
    result.height = rgb_height;

    for(int y = 0; y < rgb_height; y++){
        for(int x = 0; x < rgb_width; x++){
            DepthPixel pixel;
            pixel.color = Vector3D(lrgb_buffer[(x + y*rgb_width)*3 + 2],
                lrgb_buffer[(x + y*rgb_width)*3 + 1],
                lrgb_buffer[(x + y*rgb_width)*3 + 0]);
            pixel.have_pos = pixel_positions[x + y*rgb_width].second;
            pixel.pos = pixel_positions[x + y*rgb_width].first;
            result.depth_pixels.push_back(pixel);
        }
    }

    return result;
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