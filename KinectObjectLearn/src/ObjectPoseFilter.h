/*
 * ObjectPoseFilter.h
 *
 *  Created on: 11/12/2009
 *      Author: osushkov
 */

#ifndef OBJECTPOSEFILTER_H_
#define OBJECTPOSEFILTER_H_

#include "Util/Matrix.h"

class ObjectPoseFilter  {
  public:
    ObjectPoseFilter();
    ~ObjectPoseFilter();

    void performMotionUpdate(void);
    void performMeasurementUpdate(Matrix<float,7,1> measurement,
                                  Matrix<float,7,1> measurement_noise);

    Matrix<float,7,1> getCurrentPose(void) const {
        return current_pose;
    }

  private:

    Matrix<float,7,1> current_pose;
    Matrix<float,7,7> current_covariance;

    Matrix<float,7,1> process_noise;

};



#endif /* OBJECTPOSEFILTER_H_ */
