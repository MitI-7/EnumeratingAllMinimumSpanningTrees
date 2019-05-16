#pragma once
#include <vector>
#include <unordered_map>
#include <set>
#include <bitset>
#include "UnDirectedEdge.h"

class AllSpanningTrees {
public:
    int N = 0; // num of node
    std::vector<std::vector<int>> graph;
    std::vector<UnDirectedEdge> edges;
    std::unordered_map<int, std::vector<int>> candi;
    std::set<int> leave;
    std::vector<std::pair<int, int>> operation;

public:
    AllSpanningTrees() = default;
    AllSpanningTrees(int num_node) : N(num_node) {
        this->graph.resize(this->N);
    }

    void add_edge(int node1_name, int node2_name, int cost, int edge_name) {
        this->edges.emplace_back(UnDirectedEdge(node1_name, node1_name, node2_name, node2_name, cost, edge_name));
        const int edge_idx = (int)this->edges.size() - 1;
        this->graph[node1_name].emplace_back(edge_idx);
        this->graph[node2_name].emplace_back(edge_idx);
    }

    void set_graph(std::vector<std::vector<int>> &g) {
        this->graph = g;
    }

    void set_edges(std::vector<UnDirectedEdge> &e) {
        this->edges = e;
    }

    // O(n + e + k)
    bool build() {
        this->N = this->graph.size();
        const int root = this->edges[0].node1;

        std::unordered_set<int> visited;
        std::vector<int> st;
        find_spanning_tree(root, visited, st);

        if (not this->is_spanning_tree(st)) {
            return false;
        }

        rename_graph(st);
        assert(is_valid_graph());

        find_initial_candi();

        for (int edge_idx = 0; edge_idx < N - 1; ++edge_idx) {
            if (not candi[edge_idx].empty()) {
                this->leave.insert(edge_idx);
            }
        }

        find_child();

        return true;
    }

    // O(eklogk)
    std::set<std::vector<int>> generate_all_spanning_trees() {
        std::set<std::vector<int>> spanning_trees;

        std::vector<int> tree(this->edges.size());
        // insert T0
        for (int edge_idx = 0; edge_idx < this->N - 1; ++edge_idx) {
            tree[edge_idx] = 1;
        }
        spanning_trees.insert(tree);

        for (const auto &p : this->operation) {
            tree[p.first] = 0;
            tree[p.second] = 1;
            spanning_trees.insert(tree);
        }

        return spanning_trees;
    }

    UnDirectedEdge get_edge(int edge_idx) {
        return this->edges.at(edge_idx);
    }

private:
    // O(V + E)
    void find_spanning_tree(const int u, std::unordered_set<int> &visited, std::vector<int> &spanning_tree) {
        visited.insert(u);
        for (auto edge_idx : this->graph[u]) {
            const auto &e = this->edges[edge_idx];
            // remove self-loop
            if (e.node1 == e.node2) {
                continue;
            }

            assert(u == e.node1 or u == e.node2);
            const int v = (u == e.node1) ? e.node2 : e.node1;

            if (visited.find(v) == visited.end()) {
                spanning_tree.emplace_back(edge_idx);
                find_spanning_tree(v, visited, spanning_tree);
            }
        }
    }

    void rename_graph(const std::vector<int> &spanning_tree) {
        std::vector<int> node_rename_map(this->N, -1);
        std::vector<int> edge_rename_map(this->edges.size(), -1);
        int node = 0;
        int edge = 0;
        for (auto edge_idx : spanning_tree) {
            const auto &e = this->edges[edge_idx];

            // node
            if (node_rename_map[e.node1] == -1) {
                node_rename_map[e.node1] = node;
                node++;
            }
            if (node_rename_map[e.node2] == -1) {
                node_rename_map[e.node2] = node;
                node++;
            }

            // edge
            if (edge_rename_map[edge_idx] == -1) {
                edge_rename_map[edge_idx] = edge;
                edge++;
            }
        }

        // rename node
        for (auto &e : this->edges) {
            int node1 = node_rename_map[e.node1];
            int node2 = node_rename_map[e.node2];
            e.node1 = std::min(node1, node2);
            e.node2 = std::max(node1, node2);
        }

        // rename edge
        std::vector<std::pair<int, int>> src_edge;
        for (int edge_idx = 0; edge_idx < this->edges.size(); ++edge_idx) {
            if (edge_rename_map[edge_idx] == -1) {
                src_edge.emplace_back(std::make_pair(this->edges[edge_idx].node1, edge_idx));
            }
        }
        std::sort(src_edge.begin(), src_edge.end());
        for (const auto &p : src_edge) {
            int edge_idx = p.second;
            if (edge_rename_map[edge_idx] == -1) {
                edge_rename_map[edge_idx] = edge;
                edge++;
            }
        }

        std::vector<std::vector<int>> new_graph(this->N);
        for (int pre_node = 0; pre_node < this->graph.size(); ++pre_node) {
            for (const auto edge_idx : this->graph[pre_node]) {
                new_graph[node_rename_map[pre_node]].emplace_back(edge_rename_map[edge_idx]);
            }
            const int new_node = node_rename_map[pre_node];
            std::sort(new_graph[new_node].begin(), new_graph[new_node].end());
        }
        this->graph = new_graph;

        std::vector<UnDirectedEdge> new_edges(this->edges.size());
        for(int pre_edge_idx = 0; pre_edge_idx < this->edges.size(); ++pre_edge_idx) {
            int new_edge_idx = edge_rename_map[pre_edge_idx];
            new_edges[new_edge_idx] = this->edges[pre_edge_idx];
        }

        this->edges = new_edges;
    }

    // O(V + E)
    void find_initial_candi() {
        for (int j = 0; j < this->N - 1; ++j) {
            const auto &e_j = this->edges[j];

            for (int i : graph[e_j.node2]) {
                const auto &e = this->edges[i];

                if (is_tree0_edge(i)) {
                    continue;
                }

                if (e.node2 == e_j.node2 and e.node1 <= e_j.node1) {
                    this->candi[j].emplace_back(i);
                }
            }
        }

        for (auto &p : this->candi) {
            sort(p.second.begin(), p.second.end());
        }
    }

    void find_child() {
        if (this->leave.empty()) {
            return;
        }

        std::deque<int> que;
        const int e_k = *(--this->leave.end());
        this->leave.erase(e_k);

        while (not this->candi[e_k].empty()) {
            const auto g = this->candi[e_k].back(); this->candi[e_k].pop_back();
            que.push_front(g);

            this->operation.emplace_back(std::make_pair(e_k, g));

            sub_child(e_k, g);

            this->operation.emplace_back(std::make_pair(g, e_k));
        }

        // keep order of candi[e_k]
        while (not que.empty()) {
            this->candi[e_k].emplace_back(que.front());
            que.pop_front();
        }

        sub_child(e_k, e_k);
        this->leave.insert(e_k);
    }

    void sub_child(const int e_k, const int g) {
        if (this->candi[e_k].empty()) {
            find_child();
            return;
        }

        if (this->edges[g].node1 <= this->edges[this->candi[e_k][0]].node1) {
            find_child();
            return;
        }

        int f = -1;
        for (auto edge_idx : this->graph[this->edges[g].node1]) {
            if (is_tree0_edge(edge_idx)) {
                if (this->edges[edge_idx].node2 == this->edges[g].node1) {
                    f = edge_idx;
                    break;
                }
            }
        }
        assert(f != -1);

        if (not this->candi[f].empty()) {
            auto candi_f_backup = candi[f];
            for (auto edge_idx : candi[e_k]) {
                if (this->edges[edge_idx].node1 < this->edges[g].node1) {
                    this->candi[f].emplace_back(edge_idx);
                }
            }
            sort(this->candi[f].begin(), this->candi[f].end());

            find_child();

            this->candi[f] = candi_f_backup;
        }
        else {
            for (auto edge_idx : candi[e_k]) {
                if (this->edges[edge_idx].node1 < this->edges[g].node1) {
                    this->candi[f].emplace_back(edge_idx);
                }
            }

            this->leave.insert(f);

            find_child();

            this->leave.erase(f);

            this->candi[f].clear();
        }
    }

    bool is_tree0_edge(const int edge_idx) {
        return 0 <= edge_idx and edge_idx < this->N - 1;
    }

    // Graph satisfies the following conditions:
    // (1) a depth_from_root-first spanning tree T0 of G is given
    // (2) T0={e_0, ... , e_(v-2)}, and any edge in T0 is smaller than its proper descendants
    // (3) each vertex v is smaller than its proper descendants relative to T0
    // (4) for two edges e, f not in T0, e < f only if e.node1 <= f.node1
    // check (4)
    bool is_valid_graph() {

        // [e_0, e_1, ... , e_(N - 2)] is T0
        for (int edge_idx1 = N - 1; edge_idx1 < this->edges.size(); ++edge_idx1) {
            for (int edge_idx2 = edge_idx1 + 1; edge_idx2 < this->edges.size(); ++edge_idx2) {
                const auto &e = this->edges[edge_idx1];
                const auto &f = this->edges[edge_idx2];

                if (e.node1 > f.node1) {
                    std::cerr << "condition (4) is not satisfied" << std::endl;
                    return false;
                }
            }
        }

        return true;
    }

    bool is_spanning_tree(const std::vector<int> &tree) {
        UnionFind uf(this->graph.size() + 1);

        for (int edge_idx : tree) {
            const auto &e = this->edges[edge_idx];
            uf.union_set(e.node1, e.node2);
        }

        return uf.size(this->edges[0].node1) == this->graph.size();
    }
};