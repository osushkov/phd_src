/*
 * ObjectSnapshot.cpp
 *
 *  Created on: 11/08/2009
 *      Author: osushkov
 */


#include "ObjectSnapshot.h"
#include "ObjectSnapshotDB.h"
#include "../../Util/Common.h"
#include "../../Util/Geometry.h"
#include "../../Util/ParallelServer.h"
#include "../../Util/Timer.h"
#include "../../Util/Octree.hpp"


#include <algorithm>

struct DistanceMatrixWorkerData {
    DistanceMatrixWorkerData(const std::vector<SIFTFeature3D> &snapshot_features,
                             const std::vector<SIFTFeature3D> &scene_features,
                             std::vector<float> &distance_matrix) :
                                 snapshot_features(snapshot_features),
                                 scene_features(scene_features),
                                 distance_matrix(distance_matrix) {}


    const std::vector<SIFTFeature3D> &snapshot_features;
    const std::vector<SIFTFeature3D> &scene_features;
    std::vector<float> &distance_matrix;
};

class DistanceMatrixWorker : public ParallelExecutor {
  public:

    DistanceMatrixWorker(){};
    ~DistanceMatrixWorker(){};

    void performTask(void *task_data, unsigned rank, unsigned size){
        DistanceMatrixWorkerData *data = (DistanceMatrixWorkerData *)task_data;

        unsigned start = (rank*data->snapshot_features.size())/size;
        unsigned end = ((rank+1)*data->snapshot_features.size())/size;

        unsigned width = data->scene_features.size();
        for(unsigned i = start; i < end; i++){
            for(unsigned j = 0; j < data->scene_features.size(); j++){
                float dist0 = descr_dist_sq(&(data->snapshot_features[i].sift_feature),
                                            &(data->scene_features[j].sift_feature));

                data->distance_matrix[i*width + j] = sqrtf(dist0);
            }
        }
    }
};


struct Comp {
    bool operator()(const FeatureMatchPair &a, const FeatureMatchPair &b){
        return a.sift_pmatch > b.sift_pmatch;
    }
};

struct BasisTripletComp {
    bool operator()(const BasisTriplet &a, const BasisTriplet &b){
        return a.p_match > b.p_match;
    }
};

ObjectSnapshot::ObjectSnapshot(std::string name, unsigned id) :
    id(id), name(name), have_to_camera(false) {

    setParams(true);
}

ObjectSnapshot::ObjectSnapshot(std::string name, unsigned id, Vector3D to_camera) :
    id(id), name(name), to_camera(to_camera), have_to_camera(true) {

    setParams(true);
}

ObjectSnapshot::ObjectSnapshot(const ObjectSnapshot &that) :
    id(that.getId()), name(that.getName()), to_camera(that.getToCamera()), have_to_camera(that.haveToCamera()) {
    
    setParams(true);
    this->snapshot_features = that.snapshot_features;
    this->surface_points = that.surface_points;
}

ObjectSnapshot::~ObjectSnapshot(){

}

void ObjectSnapshot::load(std::istream &input_stream){
    input_stream.read((char*)&to_camera, sizeof(Vector3D));
    input_stream.read((char*)&have_to_camera, sizeof(bool)); 

    unsigned num_features;
    input_stream.read((char*)&num_features, sizeof(unsigned));

    snapshot_features.clear();
    for(unsigned i = 0; i < num_features; i++){
        SIFTFeature3D new_feature;
        read_feature(input_stream, new_feature.sift_feature);
        input_stream.read((char*)&(new_feature.position), sizeof(Vector3D));

        snapshot_features.push_back(new_feature);
    }

    input_stream.read((char*)&num_features, sizeof(unsigned));
    surface_points.clear();
    for(unsigned i = 0; i < num_features; i++){
        SurfacePoint new_surface_point;
        input_stream.read((char*)&(new_surface_point.pos), sizeof(Vector3D));
        input_stream.read((char*)&(new_surface_point.color), sizeof(Vector3D));

        surface_points.push_back(new_surface_point);
    }
}

void ObjectSnapshot::save(std::ostream &output_stream){
    output_stream.write((char*)&to_camera, sizeof(Vector3D));
    output_stream.write((char*)&have_to_camera, sizeof(bool)); 

    unsigned num_features = snapshot_features.size();
    output_stream.write((char*)&num_features, sizeof(unsigned));

    for(unsigned i = 0; i < snapshot_features.size(); i++){
        SIFTFeature3D cur_feature = snapshot_features[i];
        write_feature(output_stream, cur_feature.sift_feature);
        output_stream.write((char*)&cur_feature.position, sizeof(Vector3D));
    }

    num_features = surface_points.size();
    output_stream.write((char*)&num_features, sizeof(unsigned));

    for(unsigned i = 0; i < surface_points.size(); i++){
        output_stream.write((char*)&(surface_points[i].pos), sizeof(Vector3D));
        output_stream.write((char*)&(surface_points[i].color), sizeof(Vector3D));
    }
}

float ObjectSnapshot::matchScene(const std::vector<SIFTFeature3D> &scene_features,
                                 std::vector<SnapshotMatchResult> &matches,
                                 Transform &approx_transform,
                                 bool hi_accuracy){

    setParams(hi_accuracy);
    if(snapshot_features.size() < 3){
        return 0.0f;
    }
    std::vector<float> distance_matrix = buildDistanceMatrix(scene_features);
    std::vector<FeatureMatchPair> match_pairs = buildMatchPairs(scene_features, distance_matrix);
    std::vector<BasisTriplet> basis_triplets = findBasisTriplets(scene_features, match_pairs);

    if(basis_triplets.size() == 0){
        return 0.0f;
    }

    BasisTripletComp comp;
    sort(basis_triplets.begin(), basis_triplets.end(), comp);


    float best_score = 0.0f;
    std::vector<SnapshotMatchResult> best_matches;
    Transform best_approx_transform;

    unsigned num_triplets_to_check = 0;
    if(params.num_triplets_check == -1){
        num_triplets_to_check = basis_triplets.size();
    }
    else{
        num_triplets_to_check = std::min<unsigned>(basis_triplets.size(), params.num_triplets_check);
    }

    for (unsigned i = 0; i < num_triplets_to_check; i++) {
        std::vector<SnapshotMatchResult> cur_matches;
        Transform cur_approx_transform;

        float score = tryMatchScene(basis_triplets[i], scene_features,
                                    cur_matches, cur_approx_transform,
                                    distance_matrix);

        if (score > best_score) {
            best_score = score;
            best_matches = cur_matches;
            best_approx_transform = cur_approx_transform;
        }
    }

    matches = best_matches;
    approx_transform = best_approx_transform;

    return best_score;
}

void ObjectSnapshot::addFeatures(const std::vector<SIFTFeature3D> &new_features, const std::vector<SurfacePoint> &new_surface_points){
    for(unsigned i = 0; i < new_features.size(); i++){
        snapshot_features.push_back(new_features[i]);
    }

    for(unsigned i = 0; i < new_surface_points.size(); i++){
        surface_points.push_back(new_surface_points[i]);
    }
}

void ObjectSnapshot::addSurfacePoints(const std::vector<SurfacePoint> &new_surface_points){
    for(unsigned i = 0; i < new_surface_points.size(); i++){
        surface_points.push_back(new_surface_points[i]);
    }
}

void ObjectSnapshot::clearFeatures(void){
    snapshot_features.clear();
}

void ObjectSnapshot::clearSurfacePoints(void){
    surface_points.clear();
}

std::vector<float> ObjectSnapshot::buildDistanceMatrix(const std::vector<SIFTFeature3D> &scene_features){
    unsigned width = scene_features.size();
    std::vector<float> distance_matrix(width*snapshot_features.size());

    ParallelServer pserver(PSM_PARALLEL);

    DistanceMatrixWorkerData worker_data(snapshot_features, scene_features, distance_matrix);
    DistanceMatrixWorker worker_thread;
    Util::Semaphore task_sem;

    ParallelTask task(&worker_thread, &worker_data, &task_sem);
    pserver.executeParallelTask(task);
    task_sem.wait();

    return distance_matrix;
}

std::vector<FeatureMatchPair> ObjectSnapshot::buildMatchPairs(const std::vector<SIFTFeature3D> &scene_features,
                                                              std::vector<float> &distance_matrix){
    std::vector<FeatureMatchPair> match_pairs;
    for (unsigned i = 0; i < snapshot_features.size(); i++) {
        for (unsigned j = 0; j < scene_features.size(); j++) {
            FeatureMatchPair new_entry;

            new_entry.snap_feature_index = i;
            new_entry.scene_feature_index = j;
            new_entry.sift_distance = queryDistanceMatrix(i, j, distance_matrix, scene_features.size());
            new_entry.sift_pmatch = ObjectSnapshotDB::getSIFTMatchScore(new_entry.sift_distance);
            assert(new_entry.sift_pmatch >= 0.0f && new_entry.sift_pmatch <= 1.0f);

            if(new_entry.sift_pmatch > 0.5f){ // TODO make this a constant of #define.
                match_pairs.push_back(new_entry);
            }
        }
    }

    Comp comp;
    sort(match_pairs.begin(), match_pairs.end(), comp);
    return match_pairs;
}

float ObjectSnapshot::queryDistanceMatrix(unsigned snap_index, unsigned scene_index,
                                          std::vector<float> &distance_matrix, unsigned width){
    return distance_matrix[scene_index + snap_index*width];
}

float ObjectSnapshot::tryMatchScene(const BasisTriplet &basis,
                                    const std::vector<SIFTFeature3D> &scene_features,
                                    std::vector<SnapshotMatchResult> &matches,
                                    Transform &approx_transform,
                                    std::vector<float> &distance_matrix) {
    matches.clear();
    std::vector<SIFTFeature3D> transform_snapshot_features;

    std::vector<Vector3D> snapshot_basis_set, scene_basis_set;
    buildBasisSet(basis, snapshot_basis_set, scene_basis_set);

    Matrix3 snapshot_basis = Geometry::matrixFromBasisVectors(snapshot_basis_set);
    Matrix3 scene_basis = Geometry::matrixFromBasisVectors(scene_basis_set);

    Matrix3 snapshot_basis_inv;
    snapshot_basis_inv.isInverse(snapshot_basis);

    Matrix3 t = scene_basis*snapshot_basis_inv;

    approx_transform.mat = t;
    approx_transform.quaternions = Common::matrixToQuaternions(t);
    approx_transform.shift = basis.point_feature[0].second.position;
    approx_transform.secondary_shift = basis.point_feature[0].first.position;


    for(unsigned i = 0; i < snapshot_features.size(); i++){
        Vector3D tpos = snapshot_features[i].position - basis.point_feature[0].first.position;
        tpos = t*tpos;
        tpos = tpos + basis.point_feature[0].second.position;

        transform_snapshot_features.push_back(snapshot_features[i]);
        transform_snapshot_features.back().position = tpos;
    }

    float basis_ori_diff = calculateOrientationOffset(basis);
    //std::cout << "!: " << basis_ori_diff << std::endl;

    //const float max_match_dist = 2.0f;
    unsigned num_correct_matches = 0;
    float result = 0.0f;

    Octree<SIFTFeature3D> *octree = buildSnapshotOctree(scene_features);
    for(unsigned i = 0; i < transform_snapshot_features.size(); i++){
        float best_score = 0.0f;
        int best_index = -1;

        std::vector<SIFTFeature3D> nearby_scene_features;
        octree->getElementsInRegion(transform_snapshot_features[i].position,
                                    params.max_fpos_diff,
                                    nearby_scene_features);

        for(unsigned j = 0; j < nearby_scene_features.size(); j++){
            unsigned index = nearby_scene_features[j].sift_feature.category;
            float pmatch = ObjectSnapshotDB::getSIFTMatchScore(queryDistanceMatrix(i, index, distance_matrix, scene_features.size()));
            float ori_diff = nearby_scene_features[j].sift_feature.ori -
                             transform_snapshot_features[i].sift_feature.ori;
            float dist =
                (nearby_scene_features[j].position-transform_snapshot_features[i].position).length();
            assert(dist < params.max_fpos_diff);

            while(ori_diff < 0.0f){ ori_diff += 2.0f*M_PI; }
            while(ori_diff >= 2.0f*M_PI){ ori_diff -= 2.0f*M_PI; }

           // if(Geometry::minAngleDistance(basis_ori_diff, ori_diff) < params.max_fori_diff &&
            if(pmatch > params.min_fpmatch &&
               pmatch > best_score){

                best_score = pmatch;
                best_index = index;
            }
        }

        if(best_index >= 0){
            num_correct_matches++;
            result += 1;

            SnapshotMatchResult new_result;
            new_result.scene_index = best_index;
            new_result.snapshot_index = i;
            new_result.pmatch = best_score;

            matches.push_back(new_result);
        }
    }

    delete octree;

    //std::cout << result << std::endl;
    return num_correct_matches;
}

std::vector<BasisTriplet>
ObjectSnapshot::findBasisTriplets(const std::vector<SIFTFeature3D> &scene_features,
                                  const std::vector<FeatureMatchPair> &match_pairs){
    std::vector<BasisTriplet> result;

    const float min_sift_pmatch = params.min_triplet_score;
    unsigned lvl0_checks = 0, lvl1_checks = 0, lvl2_checks = 0;
    const unsigned max_checks = params.max_triplets_check;
    const unsigned max_triplets = params.max_triplets;
    const float min_triplet_area = 0.5f;

    for(unsigned v0 = 0; v0 < match_pairs.size(); v0++){
        if(lvl0_checks > max_checks){ break; }
        if(match_pairs[v0].sift_pmatch < min_sift_pmatch){ break; }

        lvl0_checks++;
        lvl1_checks = 0;
        for(unsigned v1 = v0+1; v1 < match_pairs.size(); v1++){
            if(lvl1_checks > max_checks){ break; }
            if(match_pairs[v1].sift_pmatch < min_sift_pmatch){ break; }
            if(match_pairs[v1].scene_feature_index == match_pairs[v0].scene_feature_index ||
               match_pairs[v1].snap_feature_index == match_pairs[v0].snap_feature_index){
                continue;
            }

            if(!areUniquePositions(snapshot_features[match_pairs[v1].snap_feature_index],
                                   snapshot_features[match_pairs[v0].snap_feature_index])){
                continue;
            }

            if(!areUniquePositions(scene_features[match_pairs[v1].scene_feature_index],
                                   scene_features[match_pairs[v0].scene_feature_index])){
                continue;
            }

            lvl1_checks++;
            lvl2_checks = 0;
            for(unsigned v2 = v1+1; v2 < match_pairs.size(); v2++){
                if(lvl2_checks > max_checks){ break; }
                if(match_pairs[v2].sift_pmatch < min_sift_pmatch){ break; }
                if(match_pairs[v2].scene_feature_index == match_pairs[v0].scene_feature_index ||
                   match_pairs[v2].scene_feature_index == match_pairs[v0].scene_feature_index ||
                   match_pairs[v2].snap_feature_index == match_pairs[v1].snap_feature_index   ||
                   match_pairs[v2].snap_feature_index == match_pairs[v1].snap_feature_index){
                    continue;
                }

                BasisTriplet new_triplet;
                new_triplet.point_index.push_back(v0);
                new_triplet.point_index.push_back(v1);
                new_triplet.point_index.push_back(v2);

                std::pair<SIFTFeature3D,SIFTFeature3D> p0, p1, p2;
                p0.first = snapshot_features[match_pairs[v0].snap_feature_index];
                p0.second = scene_features[match_pairs[v0].scene_feature_index];
                p1.first = snapshot_features[match_pairs[v1].snap_feature_index];
                p1.second = scene_features[match_pairs[v1].scene_feature_index];
                p2.first = snapshot_features[match_pairs[v2].snap_feature_index];
                p2.second = scene_features[match_pairs[v2].scene_feature_index];

                if(!areUniquePositions(p0.first, p1.first) ||
                   !areUniquePositions(p1.first, p2.first) ||
                   !areUniquePositions(p2.first, p0.first)){
                    continue;
                }
                if(!areUniquePositions(p0.second, p1.second) ||
                   !areUniquePositions(p1.second, p2.second) ||
                   !areUniquePositions(p2.second, p0.second)){
                    continue;
                }

                new_triplet.point_feature.push_back(p0);
                new_triplet.point_feature.push_back(p1);
                new_triplet.point_feature.push_back(p2);

                new_triplet.p_match = evalTripletPMatch(new_triplet, match_pairs);
                //std::cout << new_triplet.p_match << std::endl;
                if(new_triplet.p_match > params.min_triplet_score){
                    //std::cout << "!" << std::endl;
                    float area = evalTripletSpacialSize(new_triplet);
                    if(area > min_triplet_area){
                        result.push_back(new_triplet);
                    }

                    if(result.size() >= max_triplets){
                        return result;
                    }
                }
                lvl2_checks++;
            }
        }
    }

    return result;
}

bool ObjectSnapshot::areUniquePositions(const SIFTFeature3D &var1, const SIFTFeature3D &var2){
    return (var1.position - var2.position).length() > 0.1f; // if differ by more than a mm then unique
}

float ObjectSnapshot::evalTripletPMatch(BasisTriplet &triplet, const std::vector<FeatureMatchPair> &match_pairs){
    float sift_pmatch = 0.0f;
    for(unsigned i = 0; i < triplet.point_index.size(); i++){
        sift_pmatch += match_pairs[triplet.point_index[i]].sift_pmatch;
    }
    sift_pmatch *= 1.0f/triplet.point_index.size();


    float edge_diff = 0.0f;
    for(unsigned i = 0; i < triplet.point_feature.size(); i++){
        unsigned ni = (i+1)%triplet.point_feature.size();
        Vector3D edge0 = triplet.point_feature[i].first.position-
                         triplet.point_feature[ni].first.position;

        Vector3D edge1 = triplet.point_feature[i].second.position-
                         triplet.point_feature[ni].second.position;

        edge_diff += fabs(edge0.length() - edge1.length());///edge0.length();
    }
    edge_diff /= triplet.point_feature.size();

    float dist_pmatch = Common::normalDistribution(0.0f, params.triplet_dist_sd, edge_diff);


    float orientation_score = 0.0f;

    for (unsigned i = 0; i < triplet.point_feature.size(); i++) {
        unsigned ni = (i + 1) % triplet.point_feature.size();

        std::vector<float> snapshot_score = featureAngleMatchScore(
                triplet.point_feature[i].first.sift_feature,
                triplet.point_feature[ni].first.sift_feature);
        std::vector<float> scene_score = featureAngleMatchScore(
                triplet.point_feature[i].second.sift_feature,
                triplet.point_feature[ni].second.sift_feature);

        float cur_score = 0.0f;
        for (unsigned j = 0; j < snapshot_score.size(); j++) {
            cur_score += fabs(snapshot_score[j] - scene_score[j]);
        }
        cur_score /= snapshot_score.size();
        orientation_score += cur_score;
    }

    orientation_score /= triplet.point_feature.size();
    orientation_score = Common::normalDistribution(0.0f, params.triplet_ori_sd, orientation_score);

    return (dist_pmatch*sift_pmatch*orientation_score);
}

float ObjectSnapshot::evalTripletSpacialSize(BasisTriplet &triplet){
    // here we just return the area of the triangle. We prefer large area
    // triangles for a basis.

    // Side lengths
    float a = (triplet.point_feature[0].first.position-triplet.point_feature[1].first.position).length();
    float b = (triplet.point_feature[1].first.position-triplet.point_feature[2].first.position).length();
    float c = (triplet.point_feature[2].first.position-triplet.point_feature[0].first.position).length();

    float s = (a+b+c)/2.0f; // semi-perimeter
    return sqrtf(s*(s-a)*(s-b)*(s-c));
}

void ObjectSnapshot::buildBasisSet(const BasisTriplet &basis,
                                   std::vector<Vector3D> &snapshot_basis_set,
                                   std::vector<Vector3D> &scene_basis_set) {

    Vector3D snapshot_edge0 = basis.point_feature[1].first.position -
                              basis.point_feature[0].first.position;
    Vector3D snapshot_edge1 = basis.point_feature[2].first.position -
                              basis.point_feature[0].first.position;

    Vector3D scene_edge0 = basis.point_feature[1].second.position -
                           basis.point_feature[0].second.position;
    Vector3D scene_edge1 = basis.point_feature[2].second.position -
                           basis.point_feature[0].second.position;

    snapshot_edge0.normalise();
    snapshot_edge1.normalise();

    scene_edge0.normalise();
    scene_edge1.normalise();

    Vector3D snapshot_normal = snapshot_edge0.crossProduct(snapshot_edge1);
    Vector3D snapshot_binormal = snapshot_normal.crossProduct(snapshot_edge0);

    Vector3D scene_normal = scene_edge0.crossProduct(scene_edge1);
    Vector3D scene_binormal = scene_normal.crossProduct(scene_edge0);

    snapshot_basis_set.push_back(snapshot_normal);
    snapshot_basis_set.push_back(snapshot_binormal);
    snapshot_basis_set.push_back(snapshot_edge0);

    scene_basis_set.push_back(scene_normal);
    scene_basis_set.push_back(scene_binormal);
    scene_basis_set.push_back(scene_edge0);
}

std::vector<float> ObjectSnapshot::featureAngleMatchScore(const feature &a, const feature &b) {
    float inter_feature_angle = atan2(a.x-b.x, a.y-b.y);

    float angle0 = Geometry::minAngleDistance(a.ori, b.ori);
    float angle1 = Geometry::minAngleDistance(a.ori, inter_feature_angle);
    float angle2 = Geometry::minAngleDistance(b.ori, inter_feature_angle);

    std::vector<float> result;
    result.push_back(angle0);
    result.push_back(angle1);
    result.push_back(angle2);
    return result;
}

float ObjectSnapshot::calculateOrientationOffset(const BasisTriplet &triplet){
    float avrg = 0.0f;
    std::vector<float> src_angles, dst_angles;
    Vector2D avrg_vector(0.0f, 0.0f);

    for(unsigned i = 0; i < triplet.point_feature.size(); i++){
        src_angles.push_back(triplet.point_feature[i].first.sift_feature.ori);
        dst_angles.push_back(triplet.point_feature[i].second.sift_feature.ori);
    }

    for(unsigned i = 0; i < src_angles.size(); i++){
        float diff0 = dst_angles[i] - src_angles[i];
        avrg_vector = avrg_vector + Vector2D(cos(diff0), sin(diff0));
    }

    avrg_vector.normalise();

    float result = atan2f(avrg_vector.y, avrg_vector.x);
    while(result < 0.0f){ result += 2.0f*M_PI; }
    while(result >= 2.0f*M_PI){ result -= 2.0f*M_PI; }

    return result;
}

void ObjectSnapshot::setParams(bool hi_accuracy){
    if(hi_accuracy){
        params.num_triplets_check = -1;

        params.max_fpos_diff = 0.3f;
        params.max_fori_diff = 0.3f;
        params.min_fpmatch   = 0.8f;

        params.max_triplets       = 45;
        params.max_triplets_check = 40;

        params.min_triplet_score = 0.7f;
        params.triplet_dist_sd   = 0.4f;
        params.triplet_ori_sd    = 0.4f;
    }
    else{
        params.num_triplets_check = -1;

        params.max_fpos_diff = 1.2f;
        params.max_fori_diff = 1.0f;
        params.min_fpmatch   = 0.75f;

        params.max_triplets       = 25;
        params.max_triplets_check = 15;

        params.min_triplet_score = 0.7f;
        params.triplet_dist_sd   = 1.0f;
        params.triplet_ori_sd    = 1.0f;
    }
}


Octree<SIFTFeature3D>*
ObjectSnapshot::buildSnapshotOctree(const std::vector<SIFTFeature3D> &stereo_features){
    Vector3D min_feature(9999.0f, 9999.0f, 9999.0f), max_feature(-9999.0f, -9999.0f, -9999.0f);

    for(unsigned i = 0; i < stereo_features.size(); i++){
        if(stereo_features[i].position.x < min_feature.x){
            min_feature.x = stereo_features[i].position.x;
        }
        if(stereo_features[i].position.y < min_feature.y){
            min_feature.y = stereo_features[i].position.y;
        }
        if(stereo_features[i].position.z < min_feature.z){
            min_feature.z = stereo_features[i].position.z;
        }

        if(stereo_features[i].position.x > max_feature.x){
            max_feature.x = stereo_features[i].position.x;
        }
        if(stereo_features[i].position.y > max_feature.y){
            max_feature.y = stereo_features[i].position.y;
        }
        if(stereo_features[i].position.z > max_feature.z){
            max_feature.z = stereo_features[i].position.z;
        }
    }

    Octree<SIFTFeature3D> *octree = new Octree<SIFTFeature3D>(0.5f*(min_feature + max_feature),
                                                              (max_feature.x-min_feature.x)*1.5f,
                                                              (max_feature.y-min_feature.y)*1.5f,
                                                              (max_feature.z-min_feature.z)*1.5f,
                                                              32, 12);

    for(unsigned i = 0; i < stereo_features.size(); i++){
        SIFTFeature3D insert_feature = stereo_features[i];
        insert_feature.sift_feature.category = i;
        octree->insertElement(insert_feature.position, insert_feature);
    }

    return octree;
}