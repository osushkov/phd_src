/*
 * MinCut.h
 *
 *  Created on: 12/07/2010
 *      Author: osushkov
 */

#ifndef MINCUT_H_
#define MINCUT_H_

#include <vector>
#include <map>
#include <set>

namespace MinCut {

    struct GraphEdge;
    struct GraphNode;
    struct Graph;

    struct GraphEdge {
        GraphNode *src, *dst;
        float weight;
    };

    struct GraphNode {
        int label;
        std::map<int,GraphEdge*> edges;
    };


    struct Graph {
        std::map<int,GraphNode*> nodes;
    };

    float performMinCut(Graph &in, std::set<int> &cut_labels);
    int getNextNodeLabel(void);
    void resetNodeLabels(void);

    void test(void);

}

#endif /* MINCUT_H_ */
