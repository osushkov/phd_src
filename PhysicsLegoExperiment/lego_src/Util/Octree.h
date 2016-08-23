/*
 * Octree.h
 *
 *  Created on: 03/02/2010
 *      Author: osushkov
 */

#ifndef OCTREE_H_
#define OCTREE_H_

#include <vector>
#include "Vector3D.h"


template<class T>
class Octree {
  public:
    Octree(Vector3D centre, float width, float height, float depth,
           unsigned max_node_elements, unsigned max_node_depth);
    ~Octree();

    bool insertElement(Vector3D pos, T elem);
    void getElementsInRegion(Vector3D centre, float radius, std::vector<T> &result);

private:

    struct OctreeNode{
        Vector3D centre;
        float width, height, depth;

        std::vector<OctreeNode *> children;
        std::vector< std::pair<Vector3D,T> > elements; // only non-zero if leaf node.
        unsigned node_depth;
    };

    OctreeNode *root;

    const unsigned max_node_elements;
    const unsigned max_node_depth;


    bool insertElementIntoNode(std::pair<Vector3D,T> element, OctreeNode *node);

    bool splitNode(OctreeNode *node);
    bool pointInNode(const Vector3D &point, OctreeNode *node);

    bool freeNode(OctreeNode *node);
    bool sphereIntersectsNode(Vector3D centre, float radius, OctreeNode *node);

    void getElementsInRegionNode(Vector3D centre, float radius, OctreeNode *node,
                                 std::vector<T> &result);
};


#endif /* OCTREE_H_ */
