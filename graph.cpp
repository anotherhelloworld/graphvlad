#include <vector>
#include "graph.h"

using namespace std;

Graph::Graph(std::string filename) {
//    std::ifstream in(filename);
//    int count = 1653;
//    int index, vertex;
//    double weight;
//    for (int i = 0; i < count; i++) {
//        in >> index >> vertex >> weight;
//        AddEdge(index, vertex, weight);
//        AddEdge(vertex, index, weight);
//    }
}

void Graph::ParseLinks(std::string links) {
    ifstream in(links);
    ofstream out("graph.dot");
    out << "graph test {" << endl;
    string cur;
    getline(in, cur);
    while (getline(in, cur)) {
        cout << cur << endl;
        int link;
        int aNode;
        int bNode;
        int tabCount = 0;
        vector<int> tabs(5);
        int pos = 0;
        while (tabCount <= 4) {
            if (cur[pos] == '\t') {
                tabs[tabCount] = pos;
                cout << pos << endl;
                ++tabCount;
            }
            ++pos;
        }
        link = stoi(cur.substr(0, tabs[0]));
        aNode = stoi(cur.substr(tabs[1], tabs[2] - tabs[1]));
        bNode = stoi(cur.substr(tabs[2], tabs[3] - tabs[2]));
        out << aNode << " -- " << bNode << ";" << endl;
    }
    out << "}" << endl;
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
            int to = edge.getRight();
            if (!visited[to]) {
                double tmp = from.first + edge.getWeight();
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
    edges[index].insert(Edge(index, vertex, weight));
}

void Graph::Print() {
    for (auto i : edges) {
        for (auto j : i.second) {
            std::cout << i.first << " " << j.getRight() << " " << j.getWeight() << std::endl;
        }
    }
};