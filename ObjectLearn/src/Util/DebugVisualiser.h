/*
 * DebugVisualiser.h
 *
 *  Created on: 22/10/2009
 *      Author: osushkov
 */

#ifndef DEBUGVISUALISER_H_
#define DEBUGVISUALISER_H_

#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "../Features/StereoFeature.h"


class DebugVisualiser {
  public:
    DebugVisualiser();
    ~DebugVisualiser();

    void renderInfo(std::vector<std::pair<Vector2D,Vector2D> > matched_features,
                    IplImage *imgA, IplImage *imgB);

  private:


};


#endif /* DEBUGVISUALISER_H_ */
