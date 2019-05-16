cmake_minimum_required(VERSION 3.5)
project(EnumeratingAllMinimumSpanningTrees)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-Wall -O2")
#set(CMAKE_CXX_FLAGS "-Wall -g -fsanitize=undefined -D_GLIBCXX_DEBUG")

add_executable(EnumeratingAllMinimumSpanningTrees UnionFind.h UnDirectedEdge.h AllSpanningTrees.h AllMinimumSpanningTrees.h main.cpp)