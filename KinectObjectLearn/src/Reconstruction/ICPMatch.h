
#ifndef _ICPMatch_H_
#define _ICPMatch_H_

#include "../Util/Transform.h"
#include "../Util/Vector3D.h"
#include <vector>
#include <set>

namespace ICPMatch {

    Transform match(const std::vector<Vector3D> &src, const std::vector<Vector3D> &dst, float &err);

    void buildBuckets(const std::vector<Vector3D> &src, const float grid_size,
                      std::vector<std::vector<Vector3D> > &out_buckets,
                      std::set<unsigned> &out_filled_buckets,
                      Vector3D &corner,
                      unsigned &xsize, unsigned &ysize, unsigned &zsize);

    std::vector<Vector3D> filterPoints(const std::vector<Vector3D> &src, const float grid_size);

}

#endif
