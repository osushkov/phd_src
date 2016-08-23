
#ifndef _ExperimentResultClassifier_H_
#define _ExperimentResultClassifier_H_

#include "ExperimentResult.h"
#include <vector>
#include <iostream>

class ExperimentResultClassifier {
  public:

    static ExperimentResultClassifier& instance(void);

    void buildClassifier(const std::vector<ExperimentResult> &all_seen_results);
    int classifyExperimentResult(ExperimentResult var);

    bool load(std::ifstream &in_file);
    bool save(std::ofstream &out_file);

    bool isBuilt(void) const;
    unsigned getNumLabels(void) const;

  private:
    struct ResultCluster {
        std::vector<ExperimentResult> members;
        float dist_threshold;
        int label;
    };

    bool is_built;
    std::vector<ResultCluster> result_clusters;
    

    ExperimentResultClassifier();
    ~ExperimentResultClassifier();

    bool canInsertIntoCluster(const ResultCluster &cluster, const ExperimentResult &result);

};

#endif
