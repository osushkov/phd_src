
#ifndef _Control_H_
#define _Control_H_

#include "Util/Vector3D.h"
#include <vector>

namespace Control {
    //void initialise(void);
    //void perform(void);

    void moveArmOutOfTheWay(void);
    void moveArmToWorldPoint(Vector3D point, Vector3D approach);
    void moveArmToWorldPoint(std::vector<Vector3D> pose);

    std::vector<Vector3D> gripperEulerAnglesToOrientation(std::vector<float> euler_angles);
};

#endif

