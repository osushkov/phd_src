
#include "Match.h"
#include "Util/Common.h"
#include "Util/Geometry.h"
#include "Util/Octree.h"
#include "Util/Octree.hpp"
#include "Reconstruction/icp/icpPointToPlane.h"
#include "Reconstruction/icp/icpPointToPoint.h"
#include "Reconstruction/icp/matrix.h"
#include "Reconstruction/BestFit.h"

#include <set>


Match::Match(){

}

Match::~Match(){

}



Transform Match::doMatch(MatchData &match_data, float &err){
    std::vector<BestFitFeaturePair> fit_pairs;
    for(unsigned i = 0; i < match_data.feature_matches.size(); i++){
        BestFitFeaturePair new_pair;
        new_pair.fpos0 = match_data.dst_features[match_data.feature_matches[i].scene_index].position;
        new_pair.fpos1 = match_data.src_features[match_data.feature_matches[i].snapshot_index].position;
        new_pair.pmatch = match_data.feature_matches[i].pmatch;
        fit_pairs.push_back(new_pair);
    }

    BestFit best_fit;
    Transform result;
    err = best_fit.calculateBestFit(fit_pairs, result, false);
    std::cout << "err: " << err << std::endl;
    return result;
}
    
Transform Match::doMatchApprox(MatchData &match_data, float &err){
    Transform result = match_data.approx_transform;
    float least_error = evalTransform(result, match_data);

    for(int i = 0; i < 40; i++){
        int index0, index1, index2;

        index0 = rand()%match_data.feature_matches.size();
        while((index1 = rand()%match_data.feature_matches.size()) == index0 || 
            !areUniquePositions(match_data.src_features[match_data.feature_matches[index0].snapshot_index],
                                match_data.src_features[match_data.feature_matches[index1].snapshot_index]));

        while((index2 = rand()%match_data.feature_matches.size()) == index0 || index2 == index1 ||
            !areUniquePositions(match_data.src_features[match_data.feature_matches[index2].snapshot_index],
                                match_data.src_features[match_data.feature_matches[index1].snapshot_index]) ||
            !areUniquePositions(match_data.src_features[match_data.feature_matches[index2].snapshot_index],
                                match_data.src_features[match_data.feature_matches[index0].snapshot_index]));

        std::vector<Vector3D> src_triplet, dst_triplet;
        src_triplet.push_back(match_data.src_features[match_data.feature_matches[index0].snapshot_index].position);
        src_triplet.push_back(match_data.src_features[match_data.feature_matches[index1].snapshot_index].position);
        src_triplet.push_back(match_data.src_features[match_data.feature_matches[index2].snapshot_index].position);

        dst_triplet.push_back(match_data.dst_features[match_data.feature_matches[index0].scene_index].position);
        dst_triplet.push_back(match_data.dst_features[match_data.feature_matches[index1].scene_index].position);
        dst_triplet.push_back(match_data.dst_features[match_data.feature_matches[index2].scene_index].position);

        Transform t = buildTransform(src_triplet, dst_triplet);
        float error = evalTransform(t, match_data);
        if(error < least_error){
            least_error = error;
            result = t;
        }
    }

    std::cout << "err: " << least_error << std::endl;
    err = least_error;
    return result;
}

Transform Match::refine(std::vector<Vector3D> src_points, std::vector<Vector3D> dst_points){
    //filterPoints(src_points, dst_points);

    double *src_buffer = (double*)calloc(3*src_points.size(),sizeof(double));
    double *dst_buffer = (double*)calloc(3*dst_points.size(),sizeof(double));

    for(unsigned i = 0; i < src_points.size(); i++){
        src_buffer[i*3 + 0] = src_points[i].x;
        src_buffer[i*3 + 1] = src_points[i].y;
        src_buffer[i*3 + 2] = src_points[i].z;
    }
    
    for(unsigned i = 0; i < dst_points.size(); i++){
        dst_buffer[i*3 + 0] = dst_points[i].x;
        dst_buffer[i*3 + 1] = dst_points[i].y;
        dst_buffer[i*3 + 2] = dst_points[i].z;
    }

    IcpPointToPoint icp(dst_buffer, dst_points.size(), 3);

    ICPMatrix R = ICPMatrix::eye(3);
    ICPMatrix t(3,1);
    icp.fit(src_buffer, src_points.size(), R, t, -1);

    Transform result;
    result.secondary_shift = Vector3D(0.0f, 0.0f, 0.0f);
    result.shift = Vector3D(t.val[0][0], t.val[0][1], t.val[0][2]);

    for(unsigned i = 0; i < 3; i++){
        for(unsigned j = 0; j < 3; j++){
            result.mat(i, j) = R.val[i][j];
        }
    }

    free(src_buffer);
    free(dst_buffer);

    return result;
}

void Match::filterPoints(std::vector<Vector3D> &src_points, std::vector<Vector3D> &dst_points){
    float width = 0.0f, height = 0.0f, depth = 0.0f;
    Vector3D centre = Geometry::centreOfMass(dst_points);

    for(unsigned i = 0; i < dst_points.size(); i++){
        float xd = fabs(dst_points[i].x - centre.x);
        float yd = fabs(dst_points[i].y - centre.y);
        float zd = fabs(dst_points[i].z - centre.z);

        if(xd > width){ width = xd; }
        if(yd > height){ height = yd; }
        if(zd > depth){ depth = zd; }
    }

    Octree<unsigned> octree(centre, width*2.0f, height*2.0f, depth*2.0f, 32, 8);
    for(unsigned i = 0; i < dst_points.size(); i++){
        octree.insertElement(dst_points[i], i);
    }

    std::set<unsigned> dst_set;
    std::vector<Vector3D> rsrc, rdst;

    const float radius = 3.0f;
    for(unsigned i = 0; i < src_points.size(); i++){
        std::vector<unsigned> near_points;
        octree.getElementsInRegion(src_points[i], radius, near_points);

        if(near_points.size() > 5){
            rsrc.push_back(src_points[i]);
            for(unsigned j = 0; j < near_points.size(); j++){
                dst_set.insert(near_points[j]);
            }
        }
    }

    for(std::set<unsigned>::iterator it = dst_set.begin(); it != dst_set.end(); ++it){
        rdst.push_back(dst_points[*it]);
    }

    src_points = rsrc;
    dst_points = rdst;
}

bool Match::areUniquePositions(const SIFTFeature3D &var1, const SIFTFeature3D &var2){
    return (var1.position - var2.position).length() > 0.1f; // if differ by more than a mm then unique
}

Transform Match::buildTransform(std::vector<Vector3D> src_triplet, std::vector<Vector3D> dst_triplet){
    Vector3D src_edge0 = src_triplet[1] - src_triplet[0];
    Vector3D src_edge1 = src_triplet[2] - src_triplet[0];

    Vector3D dst_edge0 = dst_triplet[1] - dst_triplet[0];
    Vector3D dst_edge1 = dst_triplet[2] - dst_triplet[0];

    src_edge0.normalise();
    src_edge1.normalise();

    dst_edge0.normalise();
    dst_edge1.normalise();

    Vector3D src_normal = src_edge0.crossProduct(src_edge1);
    Vector3D src_binormal = src_normal.crossProduct(src_edge0);

    Vector3D dst_normal = dst_edge0.crossProduct(dst_edge1);
    Vector3D dst_binormal = dst_normal.crossProduct(dst_edge0);

    std::vector<Vector3D> src_basis_set;
    src_basis_set.push_back(src_normal);
    src_basis_set.push_back(src_binormal);
    src_basis_set.push_back(src_edge0);

    std::vector<Vector3D> dst_basis_set;
    dst_basis_set.push_back(dst_normal);
    dst_basis_set.push_back(dst_binormal);
    dst_basis_set.push_back(dst_edge0);

    Matrix3 src_basis = Geometry::matrixFromBasisVectors(src_basis_set);
    Matrix3 dst_basis = Geometry::matrixFromBasisVectors(dst_basis_set);

    Matrix3 src_basis_inv;
    src_basis_inv.isInverse(src_basis);

    Matrix3 t = dst_basis*src_basis_inv;

    Transform result;
    result.mat = t;
    result.quaternions = Common::matrixToQuaternions(t);
    result.shift = dst_triplet[0];
    result.secondary_shift = src_triplet[0];

    return result;
}

float Match::evalTransform(Transform t, MatchData &match_data){
    std::vector<SIFTFeature3D> transform_src_features;
    for(unsigned i = 0; i < match_data.src_features.size(); i++){
        SIFTFeature3D feature = match_data.src_features[i];
        feature.position = applyTransform(t, feature.position);
        transform_src_features.push_back(feature);
    }

    
    unsigned num_close = 0;
    float error = 0.0f;
    for(unsigned i = 0; i < match_data.feature_matches.size(); i++){
        SIFTFeature3D src_feature = transform_src_features[match_data.feature_matches[i].snapshot_index];
        SIFTFeature3D dst_feature = match_data.dst_features[match_data.feature_matches[i].scene_index];
        if((src_feature.position - dst_feature.position).length() < 0.1f){
            num_close++;
        }
        error += (src_feature.position - dst_feature.position).length();
    }

    //return 1.0f/(float)(num_close + 1);
    return error/(float)match_data.feature_matches.size();
}

Vector3D Match::applyTransform(Transform t, Vector3D p){
    return t.mat*(p - t.secondary_shift) + t.shift;
}