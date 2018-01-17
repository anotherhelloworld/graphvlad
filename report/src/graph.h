

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
    
    Graph();
    Graph(std::string filename);
    
    
    std::unordered_map <int, std::unordered_set <Edge, EdgeHash>> edges;
    
    
    void open(std::string filename);
    
    
    double RunDijkstraAsync();    
    
    
    void FindCriticalEdge(double k);
    
private:
    
    static const long long inf = std::numeric_limits<long long>::max();
    
    
    std::unordered_map<int, int> coord;
    std::unordered_map<int, int> coord_to_vertecies;
    
    
    std::unordered_set<int> usedEdges;
    
    
    std::vector<double> distSum;
    
    
    double Dijkstra(int v);
    
    
    void RunDijkstraThread(int from, int len);
    
    
    void AddEdge(int u, int v, double weight, int id);   
    
};

    
    class Compare {
    public:
        bool operator()(std::pair<double, int> a, std::pair<double, int> b) {
            return a.first > b.first;
        }
    };
    
