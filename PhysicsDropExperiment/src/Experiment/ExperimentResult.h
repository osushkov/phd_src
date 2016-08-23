
#ifndef _ExperimentResult_H_
#define _ExperimentResult_H_

#include <vector>
#include <fstream>
#include "../Util/Vector3D.h"
#include "../Util/Transform.h"

class ExperimentResult {
  public:
    ExperimentResult(Vector3D orientation, int label);
    ExperimentResult(Transform pose);

    ~ExperimentResult();

    Vector3D getOrientation(void) const;
    float distance(const ExperimentResult &other) const;

    int getLabel(void) const;
    void setLabel(int new_label);

    void load(std::ifstream &in_file);
    void save(std::ofstream &out_file);

  private:
    Vector3D orientation;
    int label;
};

#endif