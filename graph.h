#pragma once
#include <unordered_map>
#include <unordered_set>
#include <iostream>
#include <fstream>
#include <queue>
#include <string>
#include "edge.h"

class Graph {
public:
    //std::unordered_map <int, std::unordered_map <int, int>> edges;
    std::unordered_map <int, std::unordered_set <Edge>> edges;
    void AddEdge(int index, int vertex, double weight);
    Graph(std::string filename);
    void ParseLinks(std::string links);
    void Print();
private:
    static const long long inf = 2e9;
    int count;
    void Dijkstra(int v);
};

class Compare {
public:
    bool operator()(std::pair<double, int> a, std::pair<double, int> b) {
        return a.first > b.first;
    }
};