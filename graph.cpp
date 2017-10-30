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