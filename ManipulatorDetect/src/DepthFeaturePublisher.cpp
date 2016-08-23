
#include "DepthFeaturePublisher.h"
#include <vector>

#define FLOOR_HEIGHT 18.0f
#define MAX_HEIGHT 15.0f
#define DECAY_RATE 0.05f
#define DISPLAY_SCALE 8

#define HEIGHTMAP_SIZE 32
#define GRID_SIZE 64.0f

float heightmap[HEIGHTMAP_SIZE][HEIGHTMAP_SIZE];
char heightmap_image[HEIGHTMAP_SIZE*DISPLAY_SCALE*HEIGHTMAP_SIZE*DISPLAY_SCALE*3];

void displayHeightmap(void);
float getHeightAtPoint(float x, float y, const std::vector<Vector3D> &points);
bool isSane(Vector3D feature_pos);

void DepthFeaturePublisher::initialise(void){
    for(unsigned y = 0; y < HEIGHTMAP_SIZE; y++){
        for(unsigned x = 0; x < HEIGHTMAP_SIZE; x++){
            heightmap[y][x] = FLOOR_HEIGHT;
        }
    }
}

void DepthFeaturePublisher::submitNewFeatures(const std::vector<StereoFeature> &new_features){
    std::vector<Vector3D> world_coord_features;
    for(unsigned i = 0; i < new_features.size(); i++){
        Vector3D pos = Common::cameraPointToWorldSpace(new_features[i].position);
        if(isSane(pos)){
            world_coord_features.push_back(pos);
        }
    }

    for(unsigned y = 0; y < HEIGHTMAP_SIZE; y++){
        for(unsigned x = 0; x < HEIGHTMAP_SIZE; x++){
            float xpos = GRID_SIZE * ((float)x/HEIGHTMAP_SIZE - 0.5f);
            float ypos = GRID_SIZE * ((float)y/HEIGHTMAP_SIZE);

            float new_height = getHeightAtPoint(xpos, ypos, world_coord_features);

            if(new_height < FLOOR_HEIGHT){ new_height = FLOOR_HEIGHT; }
            if(new_height > (FLOOR_HEIGHT+MAX_HEIGHT)){ new_height = FLOOR_HEIGHT+MAX_HEIGHT; }

            heightmap[y][x] = 0.2f*new_height + 0.8f*heightmap[y][x];
        }
    }

    
    displayHeightmap();
}

void displayHeightmap(void){
    for(unsigned y = 0; y < HEIGHTMAP_SIZE*DISPLAY_SCALE; y++){
        for(unsigned x = 0; x < HEIGHTMAP_SIZE*DISPLAY_SCALE; x++){
            unsigned i = (x+y*HEIGHTMAP_SIZE*DISPLAY_SCALE)*3;
            char val = (char)(255.0f * (heightmap[HEIGHTMAP_SIZE-1-y/DISPLAY_SCALE][x/DISPLAY_SCALE]-FLOOR_HEIGHT)/MAX_HEIGHT);

            heightmap_image[i] = val;
            heightmap_image[i+1] = val;
            heightmap_image[i+2] = val;
        }
    }


    IplImage* disp = cvCreateImage(cvSize(HEIGHTMAP_SIZE*DISPLAY_SCALE, HEIGHTMAP_SIZE*DISPLAY_SCALE),IPL_DEPTH_8U,3);
    disp->imageData = heightmap_image;
    cvNamedWindow("DepthMap", 1);
    cvShowImage("DepthMap", disp);        
}

float getHeightAtPoint(float x, float y, const std::vector<Vector3D> &points){
    float height = 0.0f;

    float weight_sum = 0.0f;
    float sigma = 1.0f;

    for(unsigned i = 0; i < points.size(); i++){
        float dist2 = (x-points[i].x)*(x-points[i].x) + 
                      (y-points[i].y)*(y-points[i].y);
       
        weight_sum += expf(-dist2/(2.0f*sigma*sigma));
    }

    if(weight_sum < 0.1f){ return FLOOR_HEIGHT; }
   
    for(unsigned i = 0; i < points.size(); i++){
        float dist2 = (x-points[i].x)*(x-points[i].x) + 
                      (y-points[i].y)*(y-points[i].y);

        height += points[i].z * expf(-dist2/(2.0f*sigma*sigma))/weight_sum;
    }

    return height;
}

bool isSane(Vector3D feature_pos){
    if(feature_pos.z < FLOOR_HEIGHT/2.0f){ return false; }
    if(feature_pos.z > FLOOR_HEIGHT+2.0f*MAX_HEIGHT){ return false; }
    return true;
}

