/*
 * MinCut.cpp
 *
 *  Created on: 12/07/2010
 *      Author: osushkov
 */

#include "MinCut.h"

#include <set>
#include <map>
#include <cassert>
#include <iostream>

static int cur_node_label = 0;

static void testGraph(MinCut::Graph &graph){
    std::map<int,MinCut::GraphNode*>::iterator it;
    for(it = graph.nodes.begin(); it != graph.nodes.end(); ++it){
        //std::cout << "src: " << it->first<< std::endl;
        assert(it->first == it->second->label);

        std::map<int,MinCut::GraphEdge*>::iterator eit;
        for(eit = it->second->edges.begin(); eit != it->second->edges.end(); ++eit){
            //std::cout << "dst: " << eit->second->dst->label << std::endl;
            assert(eit->first == eit->second->dst->label);
            assert(eit->second->src == it->second);
            assert(eit->second->dst->edges.find(it->first) != eit->second->dst->edges.end());
        }
        //std::cout << "---------" << std::endl;
    }
    //std::cout << "test passed" << std::endl;
}

static std::set<int> nodeSetIntersect(std::set<int> &a, std::set<int> &b){
    std::set<int> result;

    std::set<int>::iterator it;
    for(it = a.begin(); it != a.end(); ++it){
        if(b.find(*it) != b.end()){
            result.insert(*it);
        }
    }

    return result;
}

static std::set<int> getNeighborNodes(MinCut::GraphNode *node){
    std::set<int> result;

    std::map<int,MinCut::GraphEdge*>::iterator it;
    for(it = node->edges.begin(); it != node->edges.end(); ++it){
        result.insert(it->second->dst->label);
    }

    return result;
}

static MinCut::GraphNode* mergeNodes(MinCut::Graph &graph, MinCut::GraphNode *node0, MinCut::GraphNode *node1){
    assert(node0 != NULL);
    assert(node1 != NULL);

    std::set<int> neighbor_nodes0 = getNeighborNodes(node0);
    std::set<int> neighbor_nodes1 = getNeighborNodes(node1);
    std::set<int> common_neighbors = nodeSetIntersect(neighbor_nodes0, neighbor_nodes1);

    MinCut::GraphNode *new_node = new MinCut::GraphNode;
    new_node->label = MinCut::getNextNodeLabel();

    std::set<int>::iterator it;
    for(it = neighbor_nodes0.begin(); it != neighbor_nodes0.end(); ++it){
        if(common_neighbors.find(*it) == common_neighbors.end() && *it != node1->label){
            MinCut::GraphEdge *new_edge = new MinCut::GraphEdge;
            new_edge->src = new_node;
            new_edge->dst = node0->edges[*it]->dst;
            new_edge->weight = node0->edges[*it]->weight;

            delete new_edge->dst->edges[node0->label];
            new_edge->dst->edges.erase(node0->label);

            MinCut::GraphEdge *new_dst_edge = new MinCut::GraphEdge;
            new_dst_edge->src = new_edge->dst;
            new_dst_edge->dst = new_node;
            new_dst_edge->weight = new_edge->weight;

            new_edge->dst->edges[new_node->label] = new_dst_edge;
            new_node->edges[*it] = new_edge;
        }
    }

    for(it = neighbor_nodes1.begin(); it != neighbor_nodes1.end(); ++it){
        if(common_neighbors.find(*it) == common_neighbors.end() && *it != node0->label){
            MinCut::GraphEdge *new_edge = new MinCut::GraphEdge;
            new_edge->src = new_node;
            new_edge->dst = node1->edges[*it]->dst;
            new_edge->weight = node1->edges[*it]->weight;

            delete new_edge->dst->edges[node1->label];
            new_edge->dst->edges.erase(node1->label);

            MinCut::GraphEdge *new_dst_edge = new MinCut::GraphEdge;
            new_dst_edge->src = new_edge->dst;
            new_dst_edge->dst = new_node;
            new_dst_edge->weight = new_edge->weight;

            new_edge->dst->edges[new_node->label] = new_dst_edge;
            new_node->edges[*it] = new_edge;
        }
    }

    for(it = common_neighbors.begin(); it != common_neighbors.end(); ++it){
        MinCut::GraphEdge *new_edge = new MinCut::GraphEdge;
        new_edge->src = new_node;
        new_edge->dst = node0->edges[*it]->dst;
        new_edge->weight = node0->edges[*it]->weight + node1->edges[*it]->weight;

        delete new_edge->dst->edges[node0->label];
        delete new_edge->dst->edges[node1->label];
        new_edge->dst->edges.erase(node0->label);
        new_edge->dst->edges.erase(node1->label);

        MinCut::GraphEdge *new_dst_edge = new MinCut::GraphEdge;
        new_dst_edge->src = new_edge->dst;
        new_dst_edge->dst = new_node;
        new_dst_edge->weight = new_edge->weight;

        new_edge->dst->edges[new_node->label] = new_dst_edge;
        new_node->edges[*it] = new_edge;
    }

    graph.nodes.erase(node0->label);
    graph.nodes.erase(node1->label);

    std::map<int,MinCut::GraphEdge*>::iterator eit;
    for(eit = node0->edges.begin(); eit != node0->edges.end(); ++eit){
        delete eit->second;
    }
    for(eit = node1->edges.begin(); eit != node1->edges.end(); ++eit){
        delete eit->second;
    }

    delete node0;
    delete node1;

    graph.nodes[new_node->label] = new_node;
    return new_node;
}

static float getCutCost(MinCut::GraphNode *node){
    float result = 0.0f;
    std::map<int,MinCut::GraphEdge*>::iterator it;
    for(it = node->edges.begin(); it != node->edges.end(); ++it){
        result += it->second->weight;
    }
    return result;
}

static MinCut::GraphNode* getTightestConnectedNode(MinCut::GraphNode* src){
    MinCut::GraphNode* result = NULL;
    float max_edge = 0.0f;

    std::map<int,MinCut::GraphEdge*>::iterator it;
    for(it = src->edges.begin(); it != src->edges.end(); ++it){
        if(it->second->weight > max_edge && it->second->dst->label >= 0){
            max_edge = it->second->weight;
            result = it->second->dst;
        }
    }

    return result;
}

float MinCut::performMinCut(MinCut::Graph &in, std::set<int> &cut_labels){
    testGraph(in);

    float min_cut_cost = 99999999.0f;
    MinCut::GraphNode* cur_node = in.nodes.begin()->second;

    while(in.nodes.size() > 2){
        testGraph(in);

        float cur_cut_cost = getCutCost(cur_node);
        if(cur_cut_cost < min_cut_cost){
            min_cut_cost = cur_cut_cost;
            cut_labels.clear();

            std::map<int,MinCut::GraphNode*>::iterator it;
            for(it = in.nodes.begin(); it != in.nodes.end(); ++it){
                if(it->second->label != cur_node->label){
                    cut_labels.insert(it->second->label);
                }
            }
        }

        MinCut::GraphNode* tightest_node = getTightestConnectedNode(cur_node);
        cur_node = mergeNodes(in, cur_node, tightest_node);

        testGraph(in);
    }

    if(in.nodes.begin()->second->edges.begin()->second->weight < min_cut_cost){
        min_cut_cost = in.nodes.begin()->second->edges.begin()->second->weight;
        cut_labels.clear();
        cut_labels.insert(in.nodes.begin()->second->label);
    }

    return min_cut_cost;
}

int MinCut::getNextNodeLabel(void){
    return cur_node_label++;
}

void MinCut::resetNodeLabels(void){
    cur_node_label = 0;
}

void MinCut::test(void){
    MinCut::Graph graph;

    MinCut::GraphNode *node0 = new MinCut::GraphNode; node0->label = getNextNodeLabel();
    MinCut::GraphNode *node1 = new MinCut::GraphNode; node1->label = getNextNodeLabel();
    MinCut::GraphNode *node2 = new MinCut::GraphNode; node2->label = getNextNodeLabel();
    MinCut::GraphNode *node3 = new MinCut::GraphNode; node3->label = getNextNodeLabel();
    MinCut::GraphNode *node4 = new MinCut::GraphNode; node4->label = getNextNodeLabel();
    MinCut::GraphNode *node5 = new MinCut::GraphNode; node5->label = getNextNodeLabel();
    MinCut::GraphNode *node6 = new MinCut::GraphNode; node6->label = getNextNodeLabel();
    MinCut::GraphNode *node7 = new MinCut::GraphNode; node7->label = getNextNodeLabel();

    MinCut::GraphEdge *edge01 = new MinCut::GraphEdge;
    MinCut::GraphEdge *edge10 = new MinCut::GraphEdge;

    MinCut::GraphEdge *edge12 = new MinCut::GraphEdge;
    MinCut::GraphEdge *edge21 = new MinCut::GraphEdge;

    MinCut::GraphEdge *edge23 = new MinCut::GraphEdge;
    MinCut::GraphEdge *edge32 = new MinCut::GraphEdge;

    MinCut::GraphEdge *edge34 = new MinCut::GraphEdge;
    MinCut::GraphEdge *edge43 = new MinCut::GraphEdge;

    MinCut::GraphEdge *edge45 = new MinCut::GraphEdge;
    MinCut::GraphEdge *edge54 = new MinCut::GraphEdge;

    MinCut::GraphEdge *edge56 = new MinCut::GraphEdge;
    MinCut::GraphEdge *edge65 = new MinCut::GraphEdge;

    MinCut::GraphEdge *edge67 = new MinCut::GraphEdge;
    MinCut::GraphEdge *edge76 = new MinCut::GraphEdge;

    MinCut::GraphEdge *edge70 = new MinCut::GraphEdge;
    MinCut::GraphEdge *edge07 = new MinCut::GraphEdge;

    MinCut::GraphEdge *edge16 = new MinCut::GraphEdge;
    MinCut::GraphEdge *edge61 = new MinCut::GraphEdge;

    MinCut::GraphEdge *edge25 = new MinCut::GraphEdge;
    MinCut::GraphEdge *edge52 = new MinCut::GraphEdge;

    MinCut::GraphEdge *edge35 = new MinCut::GraphEdge;
    MinCut::GraphEdge *edge53 = new MinCut::GraphEdge;

    MinCut::GraphEdge *edge17 = new MinCut::GraphEdge;
    MinCut::GraphEdge *edge71 = new MinCut::GraphEdge;


    edge01->src = node0; edge10->src = node1;
    edge01->dst = node1; edge10->dst = node0;
    edge01->weight = edge10->weight = 2.0f;

    edge12->src = node1; edge21->src = node2;
    edge12->dst = node2; edge21->dst = node1;
    edge12->weight = edge21->weight = 3.0f;

    edge23->src = node2; edge32->src = node3;
    edge23->dst = node3; edge32->dst = node2;
    edge23->weight = edge32->weight = 4.0f;

    edge34->src = node3; edge43->src = node4;
    edge34->dst = node4; edge43->dst = node3;
    edge34->weight = edge43->weight = 2.0f;

    edge45->src = node4; edge54->src = node5;
    edge45->dst = node5; edge54->dst = node4;
    edge45->weight = edge54->weight = 3.0f;

    edge56->src = node5; edge65->src = node6;
    edge56->dst = node6; edge65->dst = node5;
    edge56->weight = edge65->weight = 1.0f;

    edge67->src = node6; edge76->src = node7;
    edge67->dst = node7; edge76->dst = node6;
    edge67->weight = edge76->weight = 3.0f;

    edge07->src = node0; edge70->src = node7;
    edge07->dst = node7; edge70->dst = node0;
    edge07->weight = edge70->weight = 3.0f;

    edge16->src = node1; edge61->src = node6;
    edge16->dst = node6; edge61->dst = node1;
    edge16->weight = edge61->weight = 2.0f;

    edge25->src = node2; edge52->src = node5;
    edge25->dst = node5; edge52->dst = node2;
    edge25->weight = edge52->weight = 2.0f;

    edge35->src = node3; edge53->src = node5;
    edge35->dst = node5; edge53->dst = node3;
    edge35->weight = edge53->weight = 2.0f;

    edge17->src = node1; edge71->src = node7;
    edge17->dst = node7; edge71->dst = node1;
    edge17->weight = edge71->weight = 2.0f;


    node0->edges[1] = edge01;
    node0->edges[7] = edge07;

    node1->edges[0] = edge10;
    node1->edges[2] = edge12;
    node1->edges[6] = edge16;
    node1->edges[7] = edge17;

    node2->edges[1] = edge21;
    node2->edges[3] = edge23;
    node2->edges[5] = edge25;

    node3->edges[2] = edge32;
    node3->edges[4] = edge34;
    node3->edges[5] = edge35;

    node4->edges[3] = edge43;
    node4->edges[5] = edge45;

    node5->edges[2] = edge52;
    node5->edges[3] = edge53;
    node5->edges[4] = edge54;
    node5->edges[6] = edge56;

    node6->edges[1] = edge61;
    node6->edges[5] = edge65;
    node6->edges[7] = edge67;

    node7->edges[0] = edge70;
    node7->edges[1] = edge71;
    node7->edges[6] = edge76;


    graph.nodes[0] = node0;
    graph.nodes[1] = node1;
    graph.nodes[2] = node2;
    graph.nodes[3] = node3;
    graph.nodes[4] = node4;
    graph.nodes[5] = node5;
    graph.nodes[6] = node6;
    graph.nodes[7] = node7;

    std::set<int> result_cut;
    float cut_cost = performMinCut(graph, result_cut);
    std::cout << "min cut cost: " << cut_cost << std::endl;

    std::set<int>::iterator it;
    for(it = result_cut.begin(); it != result_cut.end(); ++it){
        std::cout << *it << " ";
    }
    std::cout << std::endl;

    std::map<int,MinCut::GraphNode*>::iterator nit;
    for(nit = graph.nodes.begin(); nit != graph.nodes.end(); ++nit){
        std::map<int,MinCut::GraphEdge*>::iterator eit;
        for(eit = nit->second->edges.begin(); eit != nit->second->edges.end(); ++eit){
            delete eit->second;
        }

        delete nit->second;
    }

}
