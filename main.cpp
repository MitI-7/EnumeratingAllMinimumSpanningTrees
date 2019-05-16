#include <bits/stdc++.h>
#include "AllMinimumSpanningTrees.h"

using namespace std;

unsigned long randxor(){
    static unsigned long x=123456789,y=362436069,z=521288629,w=88675123;
    unsigned long t;
    t=(x^(x<<11));x=y;y=z;z=w;
    return( w=(w^(w>>19))^(t^(t>>8)) );
}

set<tuple<int, int, int>> make_complete_graph(int n, int L) {
    set<tuple<int, int, int>> edges;
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            edges.insert(make_tuple(i, j, randxor() % L));
        }
    }

    return edges;
}

set<tuple<int, int, int>> make_simple_graph(int n, int m, int L) {
    set<tuple<int, int, int>> edges;

    if (m > n * (n - 1) / 2) {
        m = n * (n - 1) / 2;
    }

    for (int i = 0; i < n - 1; ++i) {
        int c = randxor() % L;
        auto t = make_tuple(i, i + 1, c);
        edges.insert(t);
    }

    for (int i = 0; i < m - n; ++i) {
        int u = randxor() % n;
        int v = randxor() % n;
        int c = randxor() % L;
        auto t = make_tuple(u, v, c);

        while (edges.find(t) != edges.end()) {
            u = randxor() % n;
            v = randxor() % n;
            t = make_tuple(u, v, c);
        }
        edges.insert(t);
    }

    return edges;
}

void sample() {
    const int num_node = 6;
    AllMinimumSpanningTrees amst(num_node);

    // add_undirected_edge(int node_name1, int node_name2, int cost, int edge_name);
    amst.add_undirected_edge(1, 2, 2, 1);
    amst.add_undirected_edge(1, 3, 1, 2);
    amst.add_undirected_edge(2, 3, 3, 3);
    amst.add_undirected_edge(2, 4, 1, 4);
    amst.add_undirected_edge(3, 4, 2, 5);
    amst.add_undirected_edge(3, 5, 2, 6);
    amst.add_undirected_edge(4, 5, 1, 7);
    amst.add_undirected_edge(4, 6, 3, 8);
    amst.add_undirected_edge(5, 6, 3, 9);

    bool ok = amst.build();
    if (not ok) {
        return;
    }

    set<vector<int>> ans = amst.generate_all_minimum_spanning_trees();
    cout << "#minimum spanning tree: " << ans.size() << endl;
    cout << "minimum cost: " << amst.minimum_cost << endl;

    int no = 1;
    for (const vector<int> &v : ans) {
        cout << "no:" << no++ << endl;
        for (int edge_idx = 0; edge_idx < v.size(); ++edge_idx) {
            if (v[edge_idx]) {
                const UnDirectedEdge &edge = amst.get_edge(edge_idx);
                cout << edge.info() << endl;
            }
        }
        cout << endl;
    }

    assert(ans.size() == 6);
    assert(amst.count() == 6);
}

void test1() {
    vector<int> expected = {0, 0, 0, 3, 16, 125, 1296, 16807, 262144, 4782969, 100000000};

    for (int n = 3; n < 11; ++n) {
        cout << "Complete graphs with constant edge weights size " << n << endl;

        AllMinimumSpanningTrees amst(n);
        auto edges = make_complete_graph(n, 1);
        auto start = std::chrono::system_clock::now();

        int no = 1;
        for (auto t : edges) {
            int u, v, c;
            tie(u, v, c) = t;
            amst.add_undirected_edge(u, v, 0, no++);
        }

        amst.build();

        auto ans = amst.generate_all_minimum_spanning_trees();
//        assert((int)ans.size() == expected[n]);
        auto end = std::chrono::system_clock::now();

        cout << " #mst:" << ans.size() << "(" << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << "sec)" << endl;
        assert(amst.count() == expected[n]);
    }
}

void test2(int n, int L) {
    cout << "Complete graphs with random edge weights size " << n << " weights uniformly distributed over [1, " << L << "]" << endl;

    const int num_test = 5;
    int num_mst = 0;
    auto start = std::chrono::system_clock::now();
    for (int i = 0; i < num_test; ++i) {
        AllMinimumSpanningTrees amst(n);

        auto edges = make_complete_graph(n, L);
        int no = 1;
        for (auto t : edges) {
            int u, v, c;
            tie(u, v, c) = t;
            amst.add_undirected_edge(u, v, c, no++);
        }

        amst.build();

        auto ans = amst.generate_all_minimum_spanning_trees();
        for (auto mst : ans) {

            // check if it is mst
            UnionFind uf(n + 100);
            int node = -1;
            int cost = 0;
            for (int edge_idx = 0; edge_idx < mst.size(); ++edge_idx) {
                if (mst[edge_idx] == 1) {
                    const auto &e = amst.get_edge(edge_idx);
                    node = e.node1;
                    cost += e.cost;
                    uf.union_set(e.node1, e.node2);
                }
            }

            assert(uf.size(node) == n);
            assert(cost == amst.minimum_cost);
        }

//        assert(ans.size() == amst.count());
        num_mst += ans.size();
    }
    auto end = std::chrono::system_clock::now();
    cout << " #mst:" << num_mst / num_test << "(" << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() / num_test << "sec)" << endl;
}

void test3(int n, int m, int L) {
    cout << "Simple graphs with random edge weights #node:" << n << " #edges:" << m << " weights uniformly distributed over [1, " << L << "]" << endl;

    const int num_test = 5;
    vector<set<tuple<int, int, int>>> edges_list;
    for (int i = 0; i < num_test; ++i) {
        edges_list.emplace_back(make_simple_graph(n, m, L));
    }

    int num_mst = 0;
    auto start = std::chrono::system_clock::now();
    for (int i = 0; i < num_test; ++i) {
        AllMinimumSpanningTrees amst(n);

        auto edges = edges_list[i];
        int no = 1;
        for (auto t : edges) {
            int u, v, c;
            tie(u, v, c) = t;
            amst.add_undirected_edge(u, v, c, no++);
        }

        amst.build();

        auto ans = amst.generate_all_minimum_spanning_trees();
        for (auto mst : ans) {

            // check if it is mst
            UnionFind uf(n + 100);
            int node = -1;
            int cost = 0;
            for (int edge_idx = 0; edge_idx < mst.size(); ++edge_idx) {
                if (mst[edge_idx] == 1) {
                    const auto &e = amst.get_edge(edge_idx);
                    node = e.node1;
                    cost += e.cost;
                    uf.union_set(e.node1, e.node2);
                }
            }

            assert(uf.size(node) == n);
            assert(cost == amst.minimum_cost);
        }

//        assert(ans.size() == amst.count());
        num_mst += ans.size();
    }
    auto end = std::chrono::system_clock::now();
    cout << " #mst:" << num_mst / num_test << "(" << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() / num_test << "sec)" << endl;
}


int main() {
    cin.tie(nullptr);
    ios::sync_with_stdio(false);

    cout << "Sample" << endl;
    sample();

    cout << "Test1" << endl;
    test1();

    cout << "Test2-1" << endl;
    for (int n = 20; n <= 60; n += 20) {
        test2(n, 100);
    }

    cout << "Test2-2" << endl;
    for (int n = 20; n <= 200; n += 20) {
        test2(n, 1000);
    }

    cout << "Test3" << endl;
    for (int n = 50; n <= 800; n *= 2) {
        test3(n, 1120, 100);
    }

    return 0;
}


