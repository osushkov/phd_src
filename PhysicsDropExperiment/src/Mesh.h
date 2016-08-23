/*
 * Mesh.h
 *
 *  Created on: 04/03/2010
 *      Author: osushkov
 */

#ifndef MESH_H_
#define MESH_H_

#include "Util/Vector3D.h"
#include <vector>

struct Mesh {
    std::vector<Vector3D> vertices;
    std::vector< std::vector<unsigned> > face_indices;
    std::vector<Vector3D> face_normals;
};

#endif /* MESH_H_ */
