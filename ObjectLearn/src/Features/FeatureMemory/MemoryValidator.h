/*
 * MemoryValidator.h
 *
 *  Created on: 02/09/2009
 *      Author: osushkov
 */

#ifndef MEMORYVALIDATOR_H_
#define MEMORYVALIDATOR_H_

#include <string>
#include <vector>

#include "../SIFT/sift.h"
#include "FeatureMemory.h"

class MemoryValidator {
  public:
    MemoryValidator();
    ~MemoryValidator();

    void load(std::string path);
    void evaluate(void);

  private:
    std::vector<IplImage*> images;
    std::vector<IplImage*> masks;

    std::vector<std::vector<feature> > features;
    std::vector<float> *cdetected, *fdetected;

    unsigned num_objects;
    const unsigned images_per_object;
    std::vector<float> params;

    bool isObjectFeature(feature &f, IplImage *mask);
    float evaluateSet(unsigned object_num, unsigned learning_set);

    float edgeDist(int x, int y, IplImage *mask, int level);
    unsigned numOnes(unsigned mask);
};


#endif /* MEMORYVALIDATOR_H_ */
