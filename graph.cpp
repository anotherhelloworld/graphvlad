#include "graph.h"

Graph::Graph(std::string filename) {
    std::ifstream in(filename);
    int count = 1653;
    int index, vertex;
    double weight;
    for (int i = 0; i < count; i++) {
        in >> index >> vertex >> weight;
        AddEdge(index, vertex, weight);
        AddEdge(vertex, index, weight);
    }
}

void Graph::AddEdge(int index, int vertex, double weight) {
    edges[index][vertex] = weight;
}

void Graph::Print() {
    for (auto i : edges) {
        for (auto j : i.second) {
            std::cout << j.first << " " << j.second << std::endl;
        }
    }
};