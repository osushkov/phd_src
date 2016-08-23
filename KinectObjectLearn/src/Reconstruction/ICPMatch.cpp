
#include "ICPMatch.h"

#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include "../Util/Octree.h"
#include "../Util/Octree.hpp"
#include "../Util/Geometry.h"


Transform ICPMatch::match(const std::vector<Vector3D> &src, const std::vector<Vector3D> &dst, float &err){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

    cloud_in->width = src.size();
    cloud_in->height = 1;
    cloud_in->is_dense = true;
    cloud_in->points.resize (cloud_in->width * cloud_in->height);
    for (unsigned i = 0; i < src.size(); ++i){
        cloud_in->points[i].x = src[i].x;
        cloud_in->points[i].y = src[i].y;
        cloud_in->points[i].z = src[i].z;
    }

    cloud_out->width = dst.size();
    cloud_out->height = 1;
    cloud_out->is_dense = true;
    cloud_out->points.resize (cloud_out->width * cloud_out->height);
    for (unsigned i = 0; i < dst.size(); ++i){
        cloud_out->points[i].x = dst[i].x;
        cloud_out->points[i].y = dst[i].y;
        cloud_out->points[i].z = dst[i].z;
    }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    //std::cout << "icp info: " << icp.getMaximumIterations() << " " << icp.getRANSACOutlierRejectionThreshold() << std::endl;
    icp.setInputCloud(cloud_in);
    icp.setInputTarget(cloud_out);
    icp.setEuclideanFitnessEpsilon(0.001);
    icp.setMaximumIterations(20);
    icp.setRANSACOutlierRejectionThreshold(2.0f);

    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    Eigen::Matrix4f rm = icp.getFinalTransformation();
    err = icp.getFitnessScore();

    Transform result;
    result.shift.x = rm(0, 3);
    result.shift.y = rm(1, 3);
    result.shift.z = rm(2, 3);

    result.secondary_shift = Vector3D(0.0f, 0.0f, 0.0f);

    for(unsigned i = 0; i < 3; i++){
        for(unsigned j = 0; j < 3; j++){
            result.mat(i, j) = rm(i, j);
        }
    }

    return result;
}

static std::vector<std::vector<Vector3D> > buckets;

void ICPMatch::buildBuckets(const std::vector<Vector3D> &src, const float grid_size,
                            std::vector<std::vector<Vector3D> > &out_buckets,
                            std::set<unsigned> &out_filled_buckets,
                            Vector3D &corner,
                            unsigned &xsize, unsigned &ysize, unsigned &zsize){

    float width = 0.0f, height = 0.0f, depth = 0.0f;
    Vector3D centre = Geometry::centreOfMass(src);

    for(unsigned i = 0; i < src.size(); i++){
        float xd = fabs(src[i].x - centre.x);
        float yd = fabs(src[i].y - centre.y);
        float zd = fabs(src[i].z - centre.z);

        if(xd > width){ width = xd; }
        if(yd > height){ height = yd; }
        if(zd > depth){ depth = zd; }
    }

    corner = centre - Vector3D(width, height, depth);

    xsize = 1 + (unsigned)(2.0f*width/grid_size);
    ysize = 1 + (unsigned)(2.0f*height/grid_size);
    zsize = 1 + (unsigned)(2.0f*depth/grid_size);

    unsigned size = xsize*ysize*zsize;
    if(out_buckets.size() < size){
        out_buckets.resize(size*4);
    }

    for(unsigned i = 0; i < src.size(); i++){
        unsigned xindex = (src[i].x - corner.x)/grid_size;
        unsigned yindex = (src[i].y - corner.y)/grid_size;
        unsigned zindex = (src[i].z - corner.z)/grid_size;
        unsigned index = xindex + yindex*xsize + zindex*xsize*ysize;
        out_filled_buckets.insert(index);
        out_buckets[index].push_back(src[i]);
    }
}

std::vector<Vector3D> ICPMatch::filterPoints(const std::vector<Vector3D> &src, const float grid_size){
    std::set<unsigned> filled_buckets;
    unsigned xsize, ysize, zsize;

    Vector3D corner;
    buildBuckets(src, grid_size, buckets, filled_buckets, corner, xsize, ysize, zsize);

    std::vector<Vector3D> result;
    std::set<unsigned>::iterator it;
    for(it = filled_buckets.begin(); it != filled_buckets.end(); ++it){
        result.push_back(Geometry::centreOfMass(buckets[*it]));
        buckets[*it].clear();
    }

    return result;

    /*
    std::vector<std::vector<Vector3D> > buckets;

    float width = 0.0f, height = 0.0f, depth = 0.0f;
    Vector3D centre = Geometry::centreOfMass(src);

    for(unsigned i = 0; i < src.size(); i++){
        float xd = fabs(src[i].x - centre.x);
        float yd = fabs(src[i].y - centre.y);
        float zd = fabs(src[i].z - centre.z);

        if(xd > width){ width = xd; }
        if(yd > height){ height = yd; }
        if(zd > depth){ depth = zd; }
    }
    Octree<unsigned> octree(centre, width, height, depth, 32, 8);

    const float radius = 0.25f;
    for(unsigned i = 0; i < src.size(); i++){
        std::vector<unsigned> near_buckets;
        octree.getElementsInRegion(src[i], radius, near_buckets);

        if(near_buckets.size() > 0){
            float nearest_dist = 10000.0f;
            unsigned nearest_index = 0;
            for(unsigned j = 0; j < near_buckets.size(); j++){
                float d = (buckets[near_buckets[j]].front() - src[i]).length2();
                if(d < nearest_dist){
                    nearest_dist = d;
                    nearest_index = near_buckets[j];
                }
            }
            buckets[nearest_index].push_back(src[i]);
        }
        else{
            std::vector<Vector3D> new_bucket;
            new_bucket.push_back(src[i]);
            buckets.push_back(new_bucket);
            octree.insertElement(src[i], buckets.size()-1);
        }
    }

    std::vector<Vector3D> result;
    for(unsigned i = 0; i < buckets.size(); i++){
        result.push_back(Geometry::centreOfMass(buckets[i]));
    }
    return result;
    */
}