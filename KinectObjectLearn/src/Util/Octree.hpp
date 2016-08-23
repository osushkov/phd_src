/*
 * Octree.cpp
 *
 *  Created on: 03/02/2010
 *      Author: osushkov
 */

#include "Octree.h"

#include <assert.h>

template<class T>
Octree<T>::Octree(Vector3D centre, float width, float height, float depth,
                  unsigned max_node_elements, unsigned max_node_depth) :
                      max_node_elements(max_node_elements), max_node_depth(max_node_depth) {

    root = new OctreeNode;

    root->centre = centre;
    root->width = width;
    root->height = height;
    root->depth = depth;

    root->node_depth = 0;
}

template<class T>
Octree<T>::~Octree(){
    freeNode(root);
}

template<class T>
bool Octree<T>::insertElement(Vector3D pos, T elem){
    if(pointInNode(pos, root)){
        return insertElementIntoNode(std::pair<Vector3D,T>(pos, elem), root);
    }
    else{
        //std::cout << "Element doesnt fit into root node" << std::endl;
        //pos.print();
        return false;
    }
}

template<class T>
void Octree<T>::getElementsInRegion(Vector3D centre, float radius, std::vector<T> &result){
    getElementsInRegionNode(centre, radius, root, result);
}

template<class T>
bool Octree<T>::insertElementIntoNode(std::pair<Vector3D,T> element, OctreeNode *node){
    assert(node != NULL);

    if(node->children.size() == 0){
        node->elements.push_back(element);
        splitNode(node);
        return true;
    }
    else{
        for(unsigned i = 0; i < node->children.size(); i++){
            if(pointInNode(element.first, node->children[i])){
                return insertElementIntoNode(element, node->children[i]);
            }
        }

        return false;
    }
}

template<class T>
bool Octree<T>::splitNode(OctreeNode *node){
    assert(node != NULL);
    assert(node->children.size() == 0);

    if(node->node_depth >= max_node_depth || node->elements.size() < max_node_elements){
        return false;
    }

    int xoffsets[] = {-1, -1, -1, -1, 1, 1, 1, 1};
    int yoffsets[] = {-1, -1, 1, 1, -1, -1, 1, 1};
    int zoffsets[] = {-1, 1, -1, 1, -1, 1, -1, 1};

    for(unsigned i = 0; i < 8; i++){
        OctreeNode *child = new OctreeNode;
        child->node_depth = node->node_depth+1;

        child->centre = node->centre;
        child->centre.x += xoffsets[i]*node->width/4.0f;
        child->centre.y += yoffsets[i]*node->height/4.0f;
        child->centre.z += zoffsets[i]*node->depth/4.0f;

        child->width = node->width/2.0f;
        child->height = node->height/2.0f;
        child->depth = node->depth/2.0f;

        std::vector< std::pair<Vector3D,T> > remaining_elements;
        for(unsigned j = 0; j < node->elements.size(); j++){
            if(pointInNode(node->elements[j].first, child)){
                child->elements.push_back(node->elements[j]);
            }
            else{
                remaining_elements.push_back(node->elements[j]);
            }
        }

        node->elements = remaining_elements;
        node->children.push_back(child);
    }

    node->elements.clear();

    for(unsigned i = 0; i < node->children.size(); i++){
        splitNode(node->children[i]);
    }

    return true;
}

template<class T>
bool Octree<T>::pointInNode(const Vector3D &point, OctreeNode *node){
    float xdiff = fabs(node->centre.x - point.x);
    float ydiff = fabs(node->centre.y - point.y);
    float zdiff = fabs(node->centre.z - point.z);

    return xdiff <= node->width/2.0f && ydiff <= node->height/2.0f && zdiff <= node->depth/2.0f;
}

template<class T>
bool Octree<T>::freeNode(OctreeNode *node){
    assert(node != NULL);

    for(unsigned i = 0; i < node->children.size(); i++){
        freeNode(node->children[i]);
    }

    delete node;
    return true;
}

template<class T>
bool Octree<T>::sphereIntersectsNode(Vector3D centre, float radius, OctreeNode *node){
    assert(node != NULL);

    float d = 0.0f;

    if(centre.x < (node->centre.x-node->width/2.0f)){
        d += (centre.x - (node->centre.x-node->width/2.0f))*
             (centre.x - (node->centre.x-node->width/2.0f));
    }
    else if(centre.x > (node->centre.x+node->width/2.0f)){
        d += (centre.x - (node->centre.x+node->width/2.0f))*
             (centre.x - (node->centre.x+node->width/2.0f));
    }

    if(centre.y < (node->centre.y-node->height/2.0f)){
        d += (centre.y - (node->centre.y-node->height/2.0f))*
             (centre.y - (node->centre.y-node->height/2.0f));
    }
    else if(centre.y > (node->centre.y+node->height/2.0f)){
        d += (centre.y - (node->centre.y+node->height/2.0f))*
             (centre.y - (node->centre.y+node->height/2.0f));
    }

    if(centre.z < (node->centre.z-node->depth/2.0f)){
        d += (centre.z - (node->centre.z-node->depth/2.0f))*
             (centre.z - (node->centre.z-node->depth/2.0f));
    }
    else if(centre.z > (node->centre.z+node->depth/2.0f)){
        d += (centre.z - (node->centre.z+node->depth/2.0f))*
             (centre.z - (node->centre.z+node->depth/2.0f));
    }

    return d < (radius*radius);
}

template<class T>
void Octree<T>::getElementsInRegionNode(Vector3D centre, float radius, OctreeNode *node,
                                        std::vector<T> &result){
    assert(node != NULL);

    if(node->children.size() > 0){
        assert(node->elements.size() == 0);
        for(unsigned i = 0; i < node->children.size(); i++){
            if(sphereIntersectsNode(centre, radius, node->children[i])){
                getElementsInRegionNode(centre, radius, node->children[i], result);
            }
        }
    }
    else{
        for(unsigned i = 0; i < node->elements.size(); i++){
            if((node->elements[i].first-centre).length() < radius){
                result.push_back(node->elements[i].second);
            }
        }
    }
}

