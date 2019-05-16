#pragma once
#include <string>

struct UnDirectedEdge {
    int edge_name;
    int node1 = -1;
    int node2 = -1;
    int node1_name;
    int node2_name;
    long long cost = 0;

    UnDirectedEdge() = default;
    UnDirectedEdge(int node1_name, int node2_name, long long cost, int edge_name) : edge_name(edge_name), cost(cost) {
        this->node1_name = std::min(node1_name, node2_name);
        this->node2_name = std::max(node1_name, node2_name);
    }
    UnDirectedEdge(int node1, int node1_name, int node2, int node2_name, long long cost, int edge_name) : edge_name(edge_name), cost(cost) {
        if (node1 < node2) {
            this->node1 = node1;
            this->node1_name = node1_name;
            this->node2 = node2;
            this->node2_name = node2_name;
        }
        else {
            this->node1 = node2;
            this->node1_name = node2_name;
            this->node2 = node1;
            this->node2_name = node1_name;
        }
    }

    std::string info() const {
        return "edge:" + std::to_string(this->edge_name) + "(" + std::to_string(this->node1_name) + "-" + std::to_string(this->node2_name) + ",cost:" + std::to_string(this->cost) + ")";
    }

    bool operator<(const UnDirectedEdge& edge) const {
        return this->cost < edge.cost;
    }

    bool operator>(const UnDirectedEdge& edge) const {
        return this->cost > edge.cost;
    }
};
