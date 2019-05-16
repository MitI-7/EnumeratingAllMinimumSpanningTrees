#pragma once
#include <vector>

class UnionFind {
private:
    std::vector<int> parent;

public:
    UnionFind(int size) : parent(size, -1) {}

    bool is_same_set(int x, int y) {
        return find_root(x) == find_root(y);
    }

    void union_set(int x, int y) {
        x = find_root(x);
        y = find_root(y);
        if (x == y) {
            return;
        }
        if (parent[x] > parent[y]) {
            std::swap(x, y);
        }

        parent[x] += parent[y];
        parent[y] = x;
    }

    unsigned int size(int x) {
        return (-parent[find_root(x)]);
    }

private:
    int find_root(int x) {
        if (parent[x] < 0) {
            return x;
        }
        else {
            return parent[x] = find_root(parent[x]);
        }
    }
};