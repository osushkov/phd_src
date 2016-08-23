
#ifndef _ExperimentResultClassifier_H_
#define _ExperimentResultClassifier_H_

#include "ExperimentResult.h"
#include <vector>
#include <iostream>

class ExperimentResultClassifier {
  public:

    static ExperimentResultClassifier& instance(void);

    void buildClassifier(void);
    int classifyExperimentResult(ExperimentResult var);
    ExperimentResult getPrototypeResult(unsigned label);

    bool isBuilt(void) const;
    unsigned getNumLabels(void) const;

  private:
    std::vector<ExperimentResult> prototype_results;
    bool is_built;
  
    ExperimentResultClassifier();
    ~ExperimentResultClassifier();
};

#endif
