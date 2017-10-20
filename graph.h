#pragma once
#include <unordered_map>
#include <iostream>
#include <fstream>

class Graph {
public:
    std::unordered_map <int, std::unordered_map <int, int>> edges;
    void AddEdge(int index, int vertex, double weight);
    Graph(std::string filename);
    void Print();
};