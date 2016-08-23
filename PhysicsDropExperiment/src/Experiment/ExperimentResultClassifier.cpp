
#include "ExperimentResultClassifier.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <list>

ExperimentResultClassifier& ExperimentResultClassifier::instance(void){
    static ExperimentResultClassifier result;
    return result;
}

ExperimentResultClassifier::ExperimentResultClassifier() : is_built(false) {

}

ExperimentResultClassifier::~ExperimentResultClassifier(){

}

void ExperimentResultClassifier::buildClassifier(const std::vector<ExperimentResult> &all_possible_results){
    std::cout << "Building Experiment Result Classifier" << std::endl;
    result_clusters.clear();

    std::list<ExperimentResult> remaining_results;
    for(unsigned i = 0; i < all_possible_results.size(); i++){
        remaining_results.push_back(all_possible_results[i]);
    }

    while(remaining_results.size() > 0){
        ResultCluster new_cluster;
        new_cluster.members.push_back(remaining_results.front());
        new_cluster.label = (int)result_clusters.size();
        new_cluster.dist_threshold = 10.0f * M_PI/180.0f;

        remaining_results.pop_front();

        bool have_updated_cluster = true;
        
        while(have_updated_cluster){
            have_updated_cluster = false;

            std::list<ExperimentResult>::iterator it = remaining_results.begin();
            while(it != remaining_results.end()){
                if(canInsertIntoCluster(new_cluster, *it)){
                    have_updated_cluster = true;
                    new_cluster.members.push_back(*it);
                    it = remaining_results.erase(it);
                }
                else{
                    ++it;
                }
            }
        }

        if(new_cluster.members.size() > 5){
            result_clusters.push_back(new_cluster);
        }
    }

    is_built = true;
}

int ExperimentResultClassifier::classifyExperimentResult(ExperimentResult var){
    float closest_dist = 0.0f;
    int best_label = -1;

    for(unsigned i = 0; i < result_clusters.size(); i++){
        for(unsigned j = 0; j < result_clusters[i].members.size(); j++){
            float d = var.distance(result_clusters[i].members[j]);
            if(best_label == -1 || d < closest_dist){
                best_label = result_clusters[i].label;
                closest_dist = d;
            }
        }
    }

    return best_label;
}

bool ExperimentResultClassifier::load(std::ifstream &in_file){
    unsigned num_clusters = 0;
    in_file >> num_clusters;
    std::cout << "Num Result Clusters: " << num_clusters << std::endl;
    result_clusters.clear();
    for(unsigned i = 0; i < num_clusters; i++){
        ResultCluster new_cluster;
        in_file >> new_cluster.dist_threshold;
        in_file >> new_cluster.label;
        
        unsigned num_members = 0;
        in_file >> num_members;

        for(unsigned j = 0; j < num_members; j++){
            ExperimentResult new_member(Vector3D(), -1);
            new_member.load(in_file);

            new_cluster.members.push_back(new_member);
        }

        result_clusters.push_back(new_cluster);
    }
    
    std::cout << "num result clusters: " << result_clusters.size() << std::endl;
    is_built = true;
    return true;
}

bool ExperimentResultClassifier::save(std::ofstream &out_file){
    out_file << result_clusters.size() << std::endl;
    for(unsigned i = 0; i < result_clusters.size(); i++){
        out_file << result_clusters[i].dist_threshold << " ";
        out_file << result_clusters[i].label << " ";
        out_file << result_clusters[i].members.size() << std::endl;

        for(unsigned j = 0; j < result_clusters[i].members.size(); j++){
            result_clusters[i].members[j].save(out_file);
        }
    }

    return true;
}

bool ExperimentResultClassifier::isBuilt(void) const {
    return is_built;
}

unsigned ExperimentResultClassifier::getNumLabels(void) const {
    return result_clusters.size();
}

bool ExperimentResultClassifier::canInsertIntoCluster(const ResultCluster &cluster, const ExperimentResult &result){
    for(unsigned i = 0; i < cluster.members.size(); i++){
        if(cluster.members[i].distance(result) < cluster.dist_threshold){
            return true;
        }
    }

    return false;
}