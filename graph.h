#pragma once
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <iostream>
#include <fstream>
#include <queue>
#include <string>
#include <atomic>
#include <thread>
#include <sstream>
#include <regex>
#include "edge.h"

class Graph {
public:
    std::unordered_map <int, std::unordered_set <Edge, EdgeHash>> edges;
    Graph(std::string filename);
    void ParseLinks(std::string links);
    void ParseLinksRegEx(std::string links);
    void Print();
    void RunDijkstraAsync();
private:
    static const long long inf = 2e17;
    int count;
    std::unordered_map<int, int> coord; //todo rename
    std::vector <std::vector <double>> dists;
    std::vector <double> Dijkstra(int v);
    void RunDijkstraThread(int from, int len);
    void AddEdge(int index, int vertex, double weight);
};

class Compare {
public:
    bool operator()(std::pair<double, int> a, std::pair<double, int> b) {
        return a.first > b.first;
    }
};