
#ifndef _ExperimentResult_H_
#define _ExperimentResult_H_

#include <vector>
#include <fstream>
#include "../Util/Vector2D.h"
#include "../Util/Transform.h"

class ExperimentResult {
  public:
    ExperimentResult(float orientation, Vector2D position, int label);
    ExperimentResult(Transform pose);
    ExperimentResult();

    ~ExperimentResult();

    float getOrientation(void) const;
    Vector2D getPosition(void) const;
    float distance(const ExperimentResult &other) const;

    int getLabel(void) const;
    void setLabel(int new_label);

    void load(std::ifstream &in_file);
    void save(std::ofstream &out_file);

  private:
    float orientation;
    Vector2D position;


    int label;
};

#endif