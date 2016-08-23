
#ifndef _ObjectFinder_H_
#define _ObjectFinder_H_

#include <vector>
#include "Features/StereoFeatureCorrelation.h"

#define HEIGHTMAP_SIZE 64


class ObjectFinder {
  public:
   
    struct ObjectOfInterest {
        std::vector<Vector3D> convex_hull;
        float average_height;
        Vector2D first_moment, second_moment;
    };


    static ObjectFinder& instance();

    void clear(void);
    void submitFeatures(const std::vector<StereoFeature> &features, 
                        std::string output_window_name);
    std::vector<ObjectOfInterest> getObjects(void);

  private:

    std::vector<ObjectOfInterest> objects_of_interest;
    float heightmap[HEIGHTMAP_SIZE][HEIGHTMAP_SIZE];
    std::string render_window_name;

    void resetHeightmap(void);
    void extractObjectsFromHeightmap(void);
    void extractObjectsFromImage(IplImage *img);
    void extractObjectsFromPixelRegions(const std::vector< std::vector<Vector2D> > &regions, 
                                        IplImage *img);

    float getHeightAtPoint(float x, float y, const std::vector<Vector3D> &points);
    bool isSane(Vector3D feature_pos);
    std::vector<Vector3D> convertHull(const std::vector<Vector2D> &hull);
        
    ObjectFinder();
    ~ObjectFinder();
};

#endif

