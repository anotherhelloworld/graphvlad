#pragma once
#include <unordered_map>
#include <iostream>
#include <fstream>
#include <queue>

class Graph {
public:
    std::unordered_map <int, std::unordered_map <int, int>> edges;
    void AddEdge(int index, int vertex, double weight);
    Graph(std::string filename);
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