Enumerating All Minimum Spanning Trees
====

## Overview
Given an weighted graph, minimum spanning trees may not be unique.    
This program lists all the minimum spanning trees included in the weighted graph.

## Usage
```
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
```

output
```
#minimum spanning tree: 6
minimum cost: 8
no:1
edge:4(2-4,cost:1)
edge:7(4-5,cost:1)
edge:2(1-3,cost:1)
edge:5(3-4,cost:2)
edge:8(4-6,cost:3)

no:2
edge:4(2-4,cost:1)
edge:7(4-5,cost:1)
edge:2(1-3,cost:1)
edge:1(1-2,cost:2)
edge:8(4-6,cost:3)

no:3
edge:6(3-5,cost:2)
edge:4(2-4,cost:1)
edge:7(4-5,cost:1)
edge:2(1-3,cost:1)
edge:8(4-6,cost:3)

no:4
edge:9(5-6,cost:3)
edge:4(2-4,cost:1)
edge:7(4-5,cost:1)
edge:2(1-3,cost:1)
edge:5(3-4,cost:2)

no:5
edge:9(5-6,cost:3)
edge:4(2-4,cost:1)
edge:7(4-5,cost:1)
edge:2(1-3,cost:1)
edge:1(1-2,cost:2)

no:6
edge:9(5-6,cost:3)
edge:6(3-5,cost:2)
edge:4(2-4,cost:1)
edge:7(4-5,cost:1)
edge:2(1-3,cost:1)
```

# Experiments
## Complete graphs with constant edge weights
|Graph|#MST's|Time(sec)|
|:---|---:|---:|
|K3|3|0|
|K4|16|0|
|K5|125|0|
|K6|1269|0|
|K7|16807|0|
|K8|262144|0|
|K9|4782969|12|
|K10|100000000|379|

## Complete graphs with random edge weights
|L|Graph|#MST's|Time(sec)|
|:---|---:|---:|---:|
|2|K20|1|2|
| |K40|48|0|
| |K60|6073|0|
|3|K20|1|0|
| |K40|1|0|
| |K60|2|0|
| |K80|2|0|
| |K100|8|0|
| |K120|65|0|
| |K140|100|0|
| |K160|3832|0|
| |K180|4115|0|
| |K200|6004|2|


# References
* [Representing all Minimum Spanning Trees with Applications to Counting and Generation](https://www.ics.uci.edu/~eppstein/pubs/Epp-TR-95-50.pdf)
* [EFFICIENTLY SCANNING ALL SPANNING TREES OF AN UNDIRECTED GRAPH](http://www.orsj.or.jp/~archive/pdf/e_mag/Vol.38_03_331.pdf)
* [ON THE PROBLEM OF FINDING ALL MINIMUM SPANNING TREES](http://www.mate.unlp.edu.ar/~liliana/lawclique_2016/07.pdf)
* [Listing all the minimum spanning trees in an undirected graph](http://www.nda.ac.jp/~yamada/paper/enum-mst.pdf)
* [Streaming SIMD Extensions - LU Decomposition](http://web.archive.org/web/20150701223512/http://download.intel.com/design/PentiumIII/sml/24504601.pdf)