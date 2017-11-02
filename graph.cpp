#include <vector>
#include "graph.h"

using namespace std;

//Public methods

Graph::Graph(std::string filename) {
    std::ifstream in(filename);
    int count = 1653;
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

void Graph::ParseLinksRegEx(std::string links) {
    ifstream in(links);
    ofstream out("graph.dot");
    out << "graph test {" << std::endl;
    string cur;
    std::regex e{ "(\\d+).*?(\\d+).*?(\\d+).*\\S" };
    std::smatch m;
    std::getline(in, cur);    
    while (std::getline(in, cur))
        if (std::regex_search(cur, m, e))
            out << m[2] << " -- " << m[3] << ";" << std::endl;
    out << "}" << std::endl;
}

void Graph::Print() {
    for (auto i : edges) {
        for (auto& j : i.second) {
            std::cout << i.first << " " << j.GetRight() << " " << j.GetWeight() << std::endl;
        }
    }
}

void Graph::RunDijkstraAsync() {
    std::atomic<int> n(0);
    int threads_count = 8; //todo replace
    dists.resize(edges.size());
    std::vector<thread> threads;
    int batch = edges.size() / threads_count;
    int remainder = edges.size() % threads_count;
    for (int i = 0; i < threads_count - 1; ++i)
        threads.push_back(thread(&Graph::RunDijkstraThread, this, i * batch, batch));
    threads.push_back(thread(&Graph::RunDijkstraThread, this, (threads_count - 1) * batch, batch + remainder));
    for (int i = 0; i < threads_count; ++i)
        threads[i].join();

    double sum = 0;
    ////std::ofstream out("dists.out");
    //FILE* output = fopen("dist.out", "w");
    for (auto i : dists) {
        for (auto j : i) {
            //fprintf(output, "%lf ", j); // fprintf faster            
            if (j != inf)
                sum += j;
            //out << j << " ";
        }
        //fprintf(output, "\n");
        //out << std::endl;
    }
    /*fclose(output);*/
    std::cout << sum << std::endl;
}

//Private methods

vector<double> Graph::Dijkstra(int v) {
    std::vector<double> dist(edges.size(), inf);
    std::vector<int> visited(edges.size(), 0);
    std::vector<int> path(edges.size(), -1);
    dist[v] = 0;
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, Compare> q;
    visited[v] = 1;
    q.push(std::make_pair(dist[v], v));

    while (!q.empty()) {
        std::pair<double, int> from = q.top(); q.pop();
        for (auto& edge : edges[from.second]) {
            int to = edge.GetRight();
            if (!visited[to]) {
                double tmp = from.first + edge.GetWeight();
                if (dist[to] > tmp) {
                    dist[to] = tmp;
                    visited[to] = 1;
                    path[to] = from.second;
                    q.push(std::make_pair(dist[to], to));
                }
            }
        }
    }
    return dist;
}

void Graph::RunDijkstraThread(int from, int len) {
    for(int i = from; i < from + len; ++i)
        dists[i] = Dijkstra(i);
}

void Graph::AddEdge(int index, int vertex, double weight) {
    edges[index].insert(Edge(index, vertex, weight));
}