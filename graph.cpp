#include "graph.h"

Graph::Graph(std::string filename) {
    std::ifstream in(filename);
    count = 1653;
    int index, vertex;
    double weight;
    for (int i = 0; i < count; i++) {
        in >> index >> vertex >> weight;
        --index;
        --vertex;
        AddEdge(index, vertex, weight);
        AddEdge(vertex, index, weight);
    }
}

void Graph::Dijkstra(int v) {
    std::vector<double> dist(edges.size(), inf);
    std::vector<int> visited(edges.size(), 0);
    std::vector<int> path(edges.size(), -1);
    dist[v] = 0;
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, Compare> q;
    visited[v] = 1;
    q.push(std::make_pair(dist[v], v));

    while (!q.empty()) {
        std::pair<double, int> from = q.top(); q.pop();
        for (auto edge : edges[from.second]) {
            int to = edge.first;
            if (!visited[to]) {
                double tmp = from.first + edge.second;
                if (dist[to] > tmp) {
                    dist[to] = tmp;
                    visited[to] = 1;
                    path[to] = from.second;
                    q.push(std::make_pair(dist[to], to));
                }
            }
        }
    }
}

void Graph::AddEdge(int index, int vertex, double weight) {
    edges[index][vertex] = weight;
}

void Graph::Print() {
    for (auto i : edges) {
        for (auto j : i.second) {
            std::cout << i.first << " " << j.first << " " << j.second << std::endl;
        }
    }
};