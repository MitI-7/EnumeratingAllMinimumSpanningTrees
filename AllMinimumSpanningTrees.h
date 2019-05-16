#pragma once

#include <algorithm>
#include <iostream>
#include <unordered_set>
#include "UnionFind.h"
#include "UnDirectedEdge.h"
#include "AllSpanningTrees.h"

class AllMinimumSpanningTrees {
public:
    long long minimum_cost;

private:
    int N;
    AllSpanningTrees ast;
    std::vector<std::vector<int>> graph;
    std::vector<UnDirectedEdge> edges;
    std::unordered_map<int, int> name_no;

    int root = -1;
    int log_v = -1;
    std::vector<std::unordered_map<int, int>> parent;      // 2^k個上の親
    std::vector<std::unordered_map<int, int>> max_cost;    // 2^k個上の親までにでてくる最大の重み
    std::vector<int> depth_from_root;
    int node_no = 0;
    bool is_constant_cost = true;
    const double EPS = 1e-7;

public:
    AllMinimumSpanningTrees(int num_node) : N(num_node){
        this->graph.resize(this->N);
    }

    void add_undirected_edge(int node_name1, int node_name2, long long cost, int edge_name) {
        if (name_no.find(node_name1) == name_no.end()) {
            name_no[node_name1] = node_no++;
        }
        if (name_no.find(node_name2) == name_no.end()) {
            name_no[node_name2] = node_no++;
        }

        int node1 = name_no[node_name1];
        int node2 = name_no[node_name2];

        this->edges.emplace_back(UnDirectedEdge(node1, node_name1, node2, node_name2, cost, edge_name));
        const int edge_idx = (int)this->edges.size() - 1;
        this->graph.at(node1).emplace_back(edge_idx);
        this->graph.at(node2).emplace_back(edge_idx);

        if (this->edges.size() > 1) {
            this->is_constant_cost &= (this->edges.back().cost == this->edges[this->edges.size() - 2].cost);
        }
    }

    // make equivalent graph(O(m logn))
    bool build() {
        const auto mst = this->find_minimum_spanning_tree();
        if (not this->is_spanning_tree(mst)) {
            std::cerr << "can't make minimum spanning tree" << std::endl;
            return false;
        }

        if (not this->is_constant_cost) {
            this->doubling(mst);
            this->graph = this->make_equivalent_graph(mst);
        }

        return true;
    }

    // count number of minimum spanning trees(O(n^3))
    long long count() {
        std::vector<std::vector<double>> matrix(this->N - 1, std::vector<double>(this->N - 1, 0));
        for (const auto &edge : this->edges) {
            int i = edge.node1;
            int j = edge.node2;
            if (i < matrix.size() and j < matrix.size()) {
                matrix[i][j]--;
                matrix[j][i]--;
            }
        }

        for (int i = 0; i < this->N - 1; ++i) {
            matrix[i][i] += this->graph[i].size();
        }

        auto res = this->determinant(matrix);
        return (long long)(res + EPS);
    }

    // construct O(n + e + k)
    // output O(eklogk)
    std::set<std::vector<int>> generate_all_minimum_spanning_trees() {
        ast.set_graph(this->graph);
        ast.set_edges(this->edges);

        bool ok = ast.build();
        if (not ok) {
            std::cerr << "can't make spanning tree" << std::endl;
            return std::set<std::vector<int>>();
        }

        return this->ast.generate_all_spanning_trees();
    }

    UnDirectedEdge get_edge(const int edge_idx) {
        return this->ast.get_edge(edge_idx);
    }

private:
    std::unordered_set<int> find_minimum_spanning_tree() {
        // sort edge_idx by cost
        std::vector<std::pair<int, int>> cost_edge;
        for (int i = 0; i < this->edges.size(); ++i) {
            const auto &e = this->edges[i];
            cost_edge.emplace_back(std::make_pair(e.cost, i));
        }
        sort(cost_edge.begin(), cost_edge.end());

        UnionFind uf(graph.size() + 1);

        std::unordered_set<int> mst;
        this->minimum_cost = 0;
        for (const auto &p : cost_edge) {
            const int edge_idx = p.second;
            const auto &e = edges[edge_idx];
            if (not uf.is_same_set(e.node1, e.node2)) {
                uf.union_set(e.node1, e.node2);
                this->minimum_cost += e.cost;
                mst.insert(edge_idx);
            }
        }

        return mst;
    }

    // O(nlogn)
    void doubling(const std::unordered_set<int> &mst) {
        // initialize
        this->root = this->edges[0].node1;
        this->log_v = int(log2(this->node_no)) + 1;
        this->parent = std::vector<std::unordered_map<int, int>>(this->log_v);
        this->max_cost = std::vector<std::unordered_map<int, int>>(this->log_v);
        this->depth_from_root.resize(this->node_no);

        // make parent, max_cost, depth_from_root
        dfs(root, -1, 0, 0, mst);
        for (int k = 0; k + 1 < log_v; k++) {
            for (int u = 0; u < this->node_no; ++u) {
                if (parent[k][u] < 0) {
                    parent[k + 1][u] = -1;
                }
                else {
                    parent[k + 1][u] = parent[k][parent[k][u]]; // uの2^k個上のノードの2^k上のノードはuの2^(k+1)個上のノード
                    if (parent[k + 1][u] >= 0) {
                        max_cost[k + 1][u] = std::max(max_cost[k][u], max_cost[k][parent[k][u]]);
                    }
                }
            }
        }
    }

    void dfs(int u, int p, int depth, int cost, const std::unordered_set<int> &mst) {
        this->parent[0][u] = p;
        this->max_cost[0][u] = cost;
        this->depth_from_root[u] = depth;

        for (int edge_idx : this->graph[u]) {
            if (mst.find(edge_idx) == mst.end()) {
                continue;
            }

            const auto &e = this->edges[edge_idx];
            const int v = e.node1 == u ? e.node2 : e.node1;

            if (v != p) {
                dfs(v, u, depth + 1, e.cost, mst);
            }
        }
    }

    int maximum_weight_ancestor(int u, int cost) {
        int d = this->depth_from_root[u];
        for (int k = this->log_v - 1; k >= 0; --k) {
            if ((1U << k) <= d) {
                if (this->max_cost[k][u] < cost) {
                    u = this->parent[k][u];
                    d = this->depth_from_root[u];
                }
            }
        }

        return u;
    }

    // O(mlogn)
    std::vector<std::vector<int>> make_equivalent_graph(const std::unordered_set<int> &mst) {
        std::vector<std::vector<int>> equivalent_graph(node_no);
        for (int edge_idx = 0; edge_idx < this->edges.size(); ++edge_idx) {
            auto &f = this->edges[edge_idx];

            const int u1 = this->maximum_weight_ancestor(f.node1, f.cost);  // destination of f.node1;
            const int u2 = this->maximum_weight_ancestor(f.node2, f.cost);  // destination of f.node2;

            // move edge f
            f.node1 = std::min(u1, u2);
            f.node2 = std::max(u1, u2);
            equivalent_graph[u1].emplace_back(edge_idx);
            equivalent_graph[u2].emplace_back(edge_idx);
        }

        return equivalent_graph;
    }

    bool is_spanning_tree(const std::unordered_set<int> &tree) {
        UnionFind uf(this->node_no);

        for (int edge_idx : tree) {
            const auto &e = this->edges[edge_idx];
            uf.union_set(e.node1, e.node2);
        }

        return uf.size(this->edges[0].node1) == this->graph.size();
    }

    // O(n^3)
    double determinant(std::vector<std::vector<double>> &matrix){
        const int n = matrix.size();
        std::vector<int> ri(n);
        std::iota(ri.begin(), ri.end(), 0);

        double det = 1.0;
        for (int p = 1 ; p <= n - 1; p++) {
            for (int i = p + 1 ; i <= n; i++) {
                if (std::abs(matrix[ri[i - 1]][p - 1]) > std::abs(matrix[ri[p - 1]][p - 1])) {
                    int t = ri[p - 1];
                    ri[p - 1] = ri[i - 1];
                    ri[i - 1] = t;
                    det = -det;
                }
            }
            if (matrix[ri[p - 1]][p - 1] == 0) {
                return false;
            }

            det = det * matrix[ri[p - 1]][p - 1];

            for (int i = p + 1 ; i <= n; i++) {
                matrix[ri[i - 1]][p - 1] /= matrix[ri[p - 1]][p - 1];

                for (int j = p + 1 ; j <= n; j++) {
                    matrix[ri[i - 1]][j - 1] -= matrix[ri[i - 1]][p - 1] * matrix[ri[p - 1]][j - 1];
                }
            }
        }

        det = det * matrix[ri[n - 1]][n - 1];
        return det;
    }
};
