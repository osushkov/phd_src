
#include "ObjectFinder.h"

#include "Util/Common.h"
#include "Util/ConvexHull.h"
#include "Util/Geometry.h"

#include <vector>

#define _USE_MATH_DEFINES
#include <math.h>

#define FLOOR_HEIGHT 11.0f
#define MAX_HEIGHT 10.0f
#define DECAY_RATE 0.05f
#define GRID_SIZE 64.0f


ObjectFinder& ObjectFinder::instance(void){
    static ObjectFinder my_finder;
    return my_finder;
}

ObjectFinder::ObjectFinder(){
    resetHeightmap();
}

ObjectFinder::~ObjectFinder(){

}


void ObjectFinder::clear(void){
    objects_of_interest.clear();
    resetHeightmap();
}


//!TODO: another version of this function that takes in a left and right image.
void ObjectFinder::submitFeatures(const std::vector<SIFTFeature3D> &new_features,
                                  std::string output_window_name){
    objects_of_interest.clear();

    render_window_name = output_window_name;
    std::vector<Vector3D> world_coord_features;

    // Convert the stereo features from camera coords to world coords.
    for(unsigned i = 0; i < new_features.size(); i++){
        Vector3D pos =
            Common::cameraPointToWorldSpace(new_features[i].position, 0.0f, -65.0f);
        if(isSane(pos)){
            world_coord_features.push_back(pos);
        }
    }

    // Update the heightmap using the new values.
    for(unsigned y = 0; y < HEIGHTMAP_SIZE; y++){
        for(unsigned x = 0; x < HEIGHTMAP_SIZE; x++){
            float xpos = GRID_SIZE * ((float)x/HEIGHTMAP_SIZE - 0.5f);
            float ypos = GRID_SIZE * ((float)y/HEIGHTMAP_SIZE);

            // TODO: this is inefficient, fix it.
            float new_height = getHeightAtPoint(xpos, ypos, world_coord_features);

            if(new_height < FLOOR_HEIGHT){ new_height = FLOOR_HEIGHT; }
            if(new_height > (FLOOR_HEIGHT+MAX_HEIGHT)){ new_height = FLOOR_HEIGHT+MAX_HEIGHT; }

            heightmap[y][x] = 0.2f*new_height + 0.8f*heightmap[y][x];
        }
    }

    objects_of_interest.clear();
    extractObjectsFromHeightmap();
    //displayHeightmap();
}


std::vector<ObjectFinder::ObjectOfInterest> ObjectFinder::getObjects(void){
    return objects_of_interest;
}


void ObjectFinder::resetHeightmap(void){
    for(unsigned y = 0; y < HEIGHTMAP_SIZE; y++){
        for(unsigned x = 0; x < HEIGHTMAP_SIZE; x++){
            heightmap[y][x] = FLOOR_HEIGHT;
        }
    }
}

void ObjectFinder::extractObjectsFromHeightmap(void){
    std::vector<unsigned char> heightmap_image_buffer(HEIGHTMAP_SIZE*HEIGHTMAP_SIZE);

    // Build the image, each pixel value is derived from the heightmap height at the
    // corresponding point.
    for(unsigned y = 0; y < HEIGHTMAP_SIZE; y++){
        for(unsigned x = 0; x < HEIGHTMAP_SIZE; x++){
            unsigned index = x+y*HEIGHTMAP_SIZE;
            float height_above_floor = heightmap[HEIGHTMAP_SIZE-1-y][x]-FLOOR_HEIGHT;
            heightmap_image_buffer.at(index) = (unsigned char)(255.0f * height_above_floor/MAX_HEIGHT);
        }
    }

    IplImage* hi_img = Common::imageFromBuffer(heightmap_image_buffer, HEIGHTMAP_SIZE, HEIGHTMAP_SIZE, 1);
    IplImage* hi_img_thresh = cvCloneImage(hi_img);

    cvThreshold(hi_img, hi_img_thresh, 60, 255, CV_THRESH_BINARY); // threshold the heightmap image
    cvDilate(hi_img_thresh, hi_img_thresh, NULL, 1);

    extractObjectsFromImage(hi_img_thresh);

    cvReleaseImage(&hi_img);
    cvReleaseImage(&hi_img_thresh);
}


void ObjectFinder::extractObjectsFromImage(IplImage *img){
    // Floodfill the thresholded image, marking each region with a
    // different colour from which we derive the region id.
    unsigned num_regions = 0;
    for(unsigned y = 0; y < HEIGHTMAP_SIZE; y++){
        for(unsigned x = 0; x < HEIGHTMAP_SIZE; x++){

            double val; Common::getPixel(img, x, y, val);
            unsigned char pix_val = (unsigned char)val;

            if(pix_val == 255){
                cvFloodFill(img, cvPoint(x, y), cvScalar(num_regions+100, 0, 0), cvScalarAll(1), cvScalarAll(1));
                num_regions++;
            }
        }
    }

    // Compute now which pixels belong to which regions from the
    // flood filled image now.
    std::vector< std::vector<Vector2D> > pixel_regions(num_regions);
    for(unsigned y = 0; y < HEIGHTMAP_SIZE; y++){
        for(unsigned x = 0; x < HEIGHTMAP_SIZE; x++){

            double val; Common::getPixel(img, x, y, val);
            unsigned char pix_val = (unsigned char)val;

            if(pix_val >= 100) {
                pixel_regions[pix_val-100].push_back(Vector2D(x, y));
            }
        }
    }


    extractObjectsFromPixelRegions(pixel_regions, img);
    cvShowImage(render_window_name.c_str(), img);
}


void ObjectFinder::extractObjectsFromPixelRegions(const std::vector< std::vector<Vector2D> > &regions, IplImage *img){
    for(unsigned i = 0; i < regions.size(); i++){
        std::vector<Vector2D> convex_hull = convexHull(regions[i]);
        Vector2D first_moment = Geometry::firstMoment(regions[i]);
        Vector2D second_moment = Geometry::secondMoment(regions[i], first_moment);

        ObjectOfInterest new_object;
        new_object.convex_hull = convertHull(convex_hull); // convert to world coordinates
        //new_object.first_moment = first_moment;
        new_object.first_moment.x = GRID_SIZE * (first_moment.x/HEIGHTMAP_SIZE - 0.5f);
        new_object.first_moment.y = GRID_SIZE * ((HEIGHTMAP_SIZE-1-first_moment.y)/HEIGHTMAP_SIZE);

        new_object.second_moment = second_moment;
        new_object.second_moment.y *= -1.0f; // from image coords to "world" coords

        objects_of_interest.push_back(new_object);

        for(unsigned j = 0; j < convex_hull.size(); j++){
            cvLine(img, cvPoint(convex_hull[j].x, convex_hull[j].y),
                   cvPoint(convex_hull[(j+1)%convex_hull.size()].x, convex_hull[(j+1)%convex_hull.size()].y),
                   CV_RGB(255, 255, 255), 1);
        }
    }
}


float ObjectFinder::getHeightAtPoint(float x, float y, const std::vector<Vector3D> &points){
    float height = 0.0f;

    float weight_sum = 0.0f;
    float sigma = 0.8f;

    for(unsigned i = 0; i < points.size(); i++){
        float dist2 = (x-points[i].x)*(x-points[i].x) +
                      (y-points[i].y)*(y-points[i].y);

        weight_sum += expf(-dist2/(2.0f*sigma*sigma));
    }

    if(weight_sum < 0.01f){ return FLOOR_HEIGHT; }

    for(unsigned i = 0; i < points.size(); i++){
        float dist2 = (x-points[i].x)*(x-points[i].x) +
                      (y-points[i].y)*(y-points[i].y);

        height += points[i].z * expf(-dist2/(2.0f*sigma*sigma))/weight_sum;
    }

    return height;
}


bool ObjectFinder::isSane(Vector3D feature_pos){
    if(feature_pos.z < FLOOR_HEIGHT/2.0f){ return false; }
    if(feature_pos.z > FLOOR_HEIGHT+2.0f*MAX_HEIGHT){ return false; }
    return true;
}


std::vector<Vector3D> ObjectFinder::convertHull(const std::vector<Vector2D> &hull){
    std::vector<Vector3D> result;
    for(unsigned i = 0; i < hull.size(); i++){
        float xpos = GRID_SIZE * (hull[i].x/HEIGHTMAP_SIZE - 0.5f);
        float ypos = GRID_SIZE * ((HEIGHTMAP_SIZE-1-hull[i].y)/HEIGHTMAP_SIZE);

        result.push_back(Vector3D(xpos, ypos, FLOOR_HEIGHT));
    }
    return result;
}

