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
#include <cfloat>

class Graph {
public:
    std::unordered_map <int, std::unordered_set <Edge, EdgeHash>> edges;
    Graph();
    Graph(std::string filename);
    void open(std::string filename);
    void ParseLinksRegEx(std::string links);
    void Print();
    double RunDijkstraAsync();
    void FindCriticalEdge(double k);
private:
    static const long long inf = 2e17;
    //int count;
    std::unordered_map<int, int> coord; //todo rename
    std::unordered_map<int, int> coord_to_vertecies;
    std::unordered_set<int> usedEdges;
    std::vector<double> distSum;
    double Dijkstra(int v);
    void RunDijkstraThread(int from, int len);
    void AddEdge(int index, int vertex, double weight, int id);

};

class Compare {
public:
    bool operator()(std::pair<double, int> a, std::pair<double, int> b) {
        return a.first > b.first;
    }
};
