/*
 * ObjectSnapshotMono.cpp
 *
 *  Created on: 25/08/2009
 *      Author: osushkov
 */


#include "ObjectSnapshotMono.h"
#include "ObjectSnapshotDBMono.h"
#include "../../Util/Common.h"
#include "../../Util/Geometry.h"

#include <algorithm>

struct Comp {
    bool operator()(const FeatureMatchPairMono &a, const FeatureMatchPairMono &b){
        return a.sift_distance < b.sift_distance;
    }
};


ObjectSnapshotMono::ObjectSnapshotMono(std::string name, unsigned id,
                                       float max_pixel_match_dist,
                                       float min_pixel_sift_match,
                                       float min_basis_pmatch,
                                       float distance_score_sd,
                                       float orientation_score_sd) :
    id(id), name(name), max_pixel_match_dist(max_pixel_match_dist),
    min_pixel_sift_match(min_pixel_sift_match), min_basis_pmatch(min_basis_pmatch),
    distance_score_sd(distance_score_sd), orientation_score_sd(orientation_score_sd) {

}

ObjectSnapshotMono::~ObjectSnapshotMono(){

}

void ObjectSnapshotMono::load(std::istream &input_stream){
    unsigned num_features;
    input_stream.read((char*)&num_features, sizeof(unsigned));

    for(unsigned i = 0; i < num_features; i++){
        feature new_feature;
        read_feature(input_stream, new_feature);
        snapshot_features.push_back(new_feature);
    }
}

void ObjectSnapshotMono::save(std::ostream &output_stream){
    unsigned num_features = snapshot_features.size();
    output_stream.write((char*)&num_features, sizeof(unsigned));

    for(unsigned i = 0; i < snapshot_features.size(); i++){
        write_feature(output_stream, snapshot_features[i]);
    }
}

float ObjectSnapshotMono::matchScene(const std::vector<feature> &scene_features,
                                     std::vector<unsigned> &matched_indexes){

    if(snapshot_features.size() < 3){ return 0.0f; }

    buildDistanceMatrix(scene_features);
    buildMatchPairs(scene_features);

    std::vector<BasisPair> basis_pairs = findBasisPairs(scene_features);
    if(basis_pairs.size() < 1){ return 0.0f; }


/*
    cvNamedWindow("Snapshot1", 1);
    IplImage* img = cvCreateImage(cvSize(512, 384),IPL_DEPTH_8U,3);
    cvSet(img, cvScalar(0,0,0));

    for(unsigned i = 0; i < scene_features.size(); i++){
        CvPoint p = cvPoint(scene_features[i].x, scene_features[i].y);
        cvCircle(img, p, 1, CV_RGB(0, 255, 0), 1);
    }

    for(unsigned i = 0; i < snapshot_features.size(); i++){
        CvPoint p = cvPoint(snapshot_features[i].x, snapshot_features[i].y);
        cvCircle(img, p, 1, CV_RGB(0, 0, 255), 1);
    }

    for(unsigned i = 0; i < basis_pairs.size(); i++){
        for(unsigned j = 0; j < basis_pairs[i].point_feature.size(); j++){
            float snap_x = basis_pairs[i].point_feature[j].first.x;
            float snap_y = basis_pairs[i].point_feature[j].first.y;

            float scene_x = basis_pairs[i].point_feature[j].second.x;
            float scene_y = basis_pairs[i].point_feature[j].second.y;

            CvPoint snap_point = cvPoint(snap_x, snap_y);
            CvPoint scene_point = cvPoint(scene_x, scene_y);

            cvLine(img, snap_point, scene_point, CV_RGB(255, 0, 0), 1);
        }
    }

    cvShowImage("Snapshot1", img);
    cvWaitKey();

    cvSet(img, cvScalar(0,0,0));
    cvReleaseImage(&img);*/

    float best_score = 0.0f;
    unsigned best_index = 0;
    for(unsigned i = 0; i < basis_pairs.size(); i++){
        matched_indexes.clear();
        float score = tryMatchScene(basis_pairs[i], scene_features, matched_indexes);
        if(score > best_score){
            best_score = score;
            best_index = i;
        }
    }

    return tryMatchScene(basis_pairs[best_index], scene_features, matched_indexes);
}

void ObjectSnapshotMono::addFeatures(const std::vector<feature> &new_features){
    for(unsigned i = 0; i < new_features.size(); i++){
        snapshot_features.push_back(new_features[i]);
    }
}

void ObjectSnapshotMono::buildDistanceMatrix(const std::vector<feature> &scene_features){
    width = scene_features.size();
    distance_matrix = std::vector<float>(width*snapshot_features.size());


    for(unsigned i = 0; i < snapshot_features.size(); i++){
        for(unsigned j = 0; j < scene_features.size(); j++){
            distance_matrix[i*width + j] =
                sqrtf(descr_dist_sq(&(snapshot_features[i]), &(scene_features[j])));
        }
    }
}

float ObjectSnapshotMono::queryDistanceMatrix(unsigned snap_index, unsigned scene_index){
    return distance_matrix[scene_index + snap_index*width];
}

void ObjectSnapshotMono::buildMatchPairs(const std::vector<feature> &scene_features){
    match_pairs.clear();
    for (unsigned i = 0; i < snapshot_features.size(); i++) {
        for (unsigned j = 0; j < scene_features.size(); j++) {
            FeatureMatchPairMono new_entry;

            new_entry.snap_feature_index = i;
            new_entry.scene_feature_index = j;
            new_entry.sift_distance = queryDistanceMatrix(i, j);
            new_entry.sift_pmatch = ObjectSnapshotDBMono::getSIFTMatchScore(new_entry.sift_distance);

            match_pairs.push_back(new_entry);
        }
    }

    Comp comp;
    sort(match_pairs.begin(), match_pairs.end(), comp);
}

float ObjectSnapshotMono::tryMatchScene(const BasisPair &basis,
                                        const std::vector<feature> &scene_features,
                                        std::vector<unsigned> &matched_indexes) {

    float avrg_orientation_change = 0.0f;
    for(unsigned i = 0; i < basis.point_feature.size(); i++){
        float diff = basis.point_feature[i].second.ori - basis.point_feature[i].first.ori;

        while(diff > M_PI){
            diff -= 2.0f*M_PI;
        }

        while(diff < -M_PI){
            diff += 2.0f*M_PI;
        }

        avrg_orientation_change += diff;
    }
    avrg_orientation_change /= basis.point_feature.size();

    std::vector<feature> transform_snapshot_features;
    Matrix2 t = buildTransformMatrix(basis);

    for(unsigned i = 0; i < snapshot_features.size(); i++){
        Vector2D tpos(snapshot_features[i].x, snapshot_features[i].y);
        tpos.x = tpos.x - basis.point_feature[0].first.x;
        tpos.y = tpos.y - basis.point_feature[0].first.y;

        tpos = t*tpos;

        tpos.x = tpos.x + basis.point_feature[0].second.x;
        tpos.y = tpos.y + basis.point_feature[0].second.y;

        feature transformed_feature = snapshot_features[i];
        transformed_feature.x = tpos.x;
        transformed_feature.y = tpos.y;
        transform_snapshot_features.push_back(transformed_feature);
    }

    for(unsigned i = 0; i < scene_features.size(); i++){
        for(unsigned j = 0; j < transform_snapshot_features.size(); j++){
            float dist = featureEuclideanDistance(scene_features[i],transform_snapshot_features[j]);
            if(dist < max_pixel_match_dist){
                float pmatch = ObjectSnapshotDBMono::getSIFTMatchScore(queryDistanceMatrix(j, i));

                float ori_diff = Geometry::minAngleDistance(
                        transform_snapshot_features[j].ori+avrg_orientation_change,scene_features[i].ori);

                //std::cout << ori_diff << "&" << transform_snapshot_features[j].ori << " "
                //          << scene_features[i].ori << std::endl;

                if(pmatch > min_pixel_sift_match/* && ori_diff < (45.0f * M_PI/180.0f) */){
                    matched_indexes.push_back(i);
                    break;
               }
            }
        }
    }

    return matched_indexes.size();
}

std::vector<BasisPair>
ObjectSnapshotMono::findBasisPairs(const std::vector<feature> &scene_features){
    std::vector<BasisPair> result;

    const float min_sift_pmatch = 0.8f;
    unsigned lvl0_checks = 0, lvl1_checks = 0, lvl2_checks = 0;
    const unsigned max_checks = 5;
    const unsigned max_pairs = 20;
    const float min_pair_length = 5.0f;

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
            for(unsigned v2 = v1 + 1; v2 < match_pairs.size(); v2++) {
                if(lvl2_checks > max_checks){ break; }
                if(match_pairs[v2].sift_pmatch < min_sift_pmatch){ break; }
                if(match_pairs[v2].scene_feature_index == match_pairs[v0].scene_feature_index ||
                   match_pairs[v2].scene_feature_index == match_pairs[v0].scene_feature_index ||
                   match_pairs[v2].snap_feature_index == match_pairs[v1].snap_feature_index   ||
                   match_pairs[v2].snap_feature_index == match_pairs[v1].snap_feature_index){
                    continue;
                }

                BasisPair new_pair;
                new_pair.point_index.push_back(v0);
                new_pair.point_index.push_back(v1);
                new_pair.point_index.push_back(v2);

                std::pair<feature, feature> p0, p1, p2;
                p0.first = snapshot_features[match_pairs[v0].snap_feature_index];
                p0.second = scene_features[match_pairs[v0].scene_feature_index];

                p1.first = snapshot_features[match_pairs[v1].snap_feature_index];
                p1.second = scene_features[match_pairs[v1].scene_feature_index];

                p2.first = snapshot_features[match_pairs[v2].snap_feature_index];
                p2.second = scene_features[match_pairs[v2].scene_feature_index];

                if(!areUniquePositions(p0.first, p1.first) ||
                   !areUniquePositions(p1.first, p2.first) ||
                   !areUniquePositions(p2.first, p0.first)) {
                    continue;
                }
                if(!areUniquePositions(p0.second, p1.second) ||
                   !areUniquePositions(p1.second, p2.second) ||
                   !areUniquePositions(p2.second, p0.second)) {
                    continue;
                }

                new_pair.point_feature.push_back(p0);
                new_pair.point_feature.push_back(p1);
                new_pair.point_feature.push_back(p2);

                new_pair.p_match = evalPairPMatch(new_pair);
                if (new_pair.p_match > min_basis_pmatch) {
                    float length = evalPairSpacialSize(new_pair);
                    if (length > min_pair_length) {
                        result.push_back(new_pair);
                    }

                    if (result.size() >= max_pairs) {
                        return result;
                    }
                }

                lvl2_checks++;
            }
        }
    }

    return result;
}

bool ObjectSnapshotMono::areUniquePositions(const feature &var1, const feature &var2){
    return (var1.x-var2.x)*(var1.x-var2.x) + (var1.y-var2.y)*(var1.y-var2.y) > 1.0f;
}

float ObjectSnapshotMono::evalPairPMatch(const BasisPair &pair){
    float sift_pmatch = 0.0f;
    for(unsigned i = 0; i < pair.point_index.size(); i++){
        sift_pmatch += match_pairs[pair.point_index[i]].sift_pmatch;
    }
    sift_pmatch *= 1.0f/pair.point_index.size();

    float distance_score = 0.0f;
    float snapshot_edge_sum = 0.0f, scene_edge_sum = 0.0f;
    for(unsigned i = 0; i < pair.point_feature.size(); i++){
            unsigned ni = (i+1)%pair.point_feature.size();
            snapshot_edge_sum += featureEuclideanDistance(pair.point_feature[i].first,
                                                          pair.point_feature[ni].first);
            scene_edge_sum += featureEuclideanDistance(pair.point_feature[i].second,
                                                       pair.point_feature[ni].second);
    }

    for(unsigned i = 0; i < pair.point_feature.size(); i++){
        unsigned ni = (i+1)%pair.point_feature.size();
        float snapshot_edge = featureEuclideanDistance(pair.point_feature[i].first,
                                                       pair.point_feature[ni].first);
        float scene_edge = featureEuclideanDistance(pair.point_feature[i].second,
                                                    pair.point_feature[ni].second);

        snapshot_edge /= snapshot_edge_sum;
        scene_edge /= scene_edge_sum;
        distance_score += fabs(snapshot_edge - scene_edge);
    }

    distance_score = Common::normalDistribution(0.0f, distance_score_sd, distance_score);

    float orientation_score = 0.0f;
    for(unsigned i = 0; i < pair.point_feature.size(); i++){
        unsigned ni = (i+1)%pair.point_feature.size();

        std::vector<float> snapshot_score =
            featureAngleMatchScore(pair.point_feature[i].first, pair.point_feature[ni].first);
        std::vector<float> scene_score =
            featureAngleMatchScore(pair.point_feature[i].second, pair.point_feature[ni].second);

        float cur_score = 0.0f;
        for(unsigned j = 0; j < snapshot_score.size(); j++){
            cur_score += fabs(snapshot_score[j] - scene_score[j]);
        }
        cur_score /= snapshot_score.size();
        orientation_score += cur_score;
    }
    orientation_score /= pair.point_feature.size();
    orientation_score = Common::normalDistribution(0.0f, orientation_score_sd, orientation_score);

    return (sift_pmatch*distance_score*orientation_score);
}

float ObjectSnapshotMono::evalPairSpacialSize(const BasisPair &pair){
    // here we just return the length of the line between the SIFT points
    return featureEuclideanDistance(pair.point_feature[0].first, pair.point_feature[1].first);
}

float ObjectSnapshotMono::featureEuclideanDistance(const feature &a, const feature &b){
    float xdiff = a.x - b.x;
    float ydiff = a.y - b.y;
    return sqrtf(xdiff*xdiff + ydiff*ydiff);
}

Matrix2 ObjectSnapshotMono::buildTransformMatrix(const BasisPair &basis){
    float snap_angle0 = atan2(basis.point_feature[1].first.x-basis.point_feature[0].first.x,
                              basis.point_feature[1].first.y-basis.point_feature[0].first.y);
    float snap_angle1 = atan2(basis.point_feature[2].first.x-basis.point_feature[0].first.x,
                              basis.point_feature[2].first.y-basis.point_feature[0].first.y);

    float scene_angle0 = atan2(basis.point_feature[1].second.x-basis.point_feature[0].second.x,
                                  basis.point_feature[1].second.y-basis.point_feature[0].second.y);
    float scene_angle1 = atan2(basis.point_feature[2].second.x-basis.point_feature[0].second.x,
                               basis.point_feature[2].second.y-basis.point_feature[0].second.y);

    float diff0 = scene_angle0 - snap_angle0;
    float diff1 = scene_angle1 - snap_angle1;
    float rot = (diff0+diff1)/2.0f;

    Matrix2 result;

    result(0, 0) = cos(rot);
    result(0, 1) = sin(rot);
    result(1, 0) = -sin(rot);
    result(1, 1) = cos(rot);

    return result;
}

std::vector<float> ObjectSnapshotMono::featureAngleMatchScore(const feature &a, const feature &b) {
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
