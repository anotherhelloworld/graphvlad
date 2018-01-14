#include <vector>
#include "graph.h"

using namespace std;

//Public methods

Graph::Graph() { }

Graph::Graph(std::string filename) {
    open(filename);
}

void Graph::open(std::string filename) {
    if (filename != "") {
        std::ifstream in(filename);
        int index, vertex;
        double weight;
        int n = 0;
        while (!in.eof()) {
            int v;
            int u;
            in >> index >> vertex >> weight;
            if (coord.find(index) != coord.end()) {
                v = coord[index];
            }
            else {
                v = n++;
                coord.insert(std::make_pair(index, v));
            }
            if (coord.find(vertex) != coord.end()) {
                u = coord[vertex];
            }
            else {
                u = n++;
                coord.insert(std::make_pair(vertex, u));
            }
            AddEdge(u, v, weight);
            AddEdge(v, u, weight);
        }

        for (auto &i : coord) {
            coord_to_vertecies[i.second] = i.first;
        }
    }
    //buff.resize(edges.size(), inf);
    dists.resize(edges.size());
    for (int i = 0; i < edges.size(); ++i)
        Dijkstra(i);
    //oldDists.resize(edges.size());
    //for (auto &i : edges) 
    //    for (auto &j : i.second) 
    //        dists[i.first][j.GetRight()] = inf;
}

void Graph::ParseLinksRegEx(std::string links) {
    ifstream in(links);
    ofstream out("vl-grand.dat");
    //out << "graph test {" << std::endl;
    string cur;
    std::regex e{ "(\\d+).*?\\t(\\d+)\\t.*?(\\d+).*?(\\d+\\.?\\d*).*\\S" };
    std::smatch m;
    std::getline(in, cur);
    int count = 0;
    while (std::getline(in, cur))
        if (std::regex_search(cur, m, e)) {
            //cout << count++ << endl;
            out << m[2] << " " << m[3] << " " << m[4] << "\n";
            //AddEdge(stoi(m[3]) - 1, stoi(m[2]) - 1, stod(m[4]));
            //AddEdge(stoi(m[2]) - 1, stoi(m[3]) - 1, stod(m[4]));
        }
    //out << "}" << std::endl;
}

void Graph::Print() {
    for (auto &i : edges) {
        for (auto& j : i.second) {
            std::cout << i.first << " " << j.GetRight() << " " << j.GetWeight() << std::endl;
        }
    }
}

double Graph::RunDijkstraAsync() {
    std::atomic<int> n(0);
    int threads_count = 8; //todo replace

    for (int i = 0; i < dists.size(); ++i)
        for (auto &j : dists[i])
            j.second = inf;

    std::vector<thread> threads;
    int batch = edges.size() / threads_count;
    int remainder = edges.size() % threads_count;
    //auto t1 = std::chrono::high_resolution_clock::now();                                           //time evaluation
    for (int i = 0; i < threads_count - 1; ++i)
        threads.push_back(thread(&Graph::RunDijkstraThread, this, i * batch, batch));
    threads.push_back(thread(&Graph::RunDijkstraThread, this, (threads_count - 1) * batch, batch + remainder));
    for (int i = 0; i < threads_count; ++i)
        threads[i].join();
    //auto t2 = std::chrono::high_resolution_clock::now();                                           //time evaluation
    //cout << chrono::duration_cast<std::chrono::microseconds>(t2-t1).count() / 1e6 << " seconds\n"; //time evaluation

    double sum = 0;
    for (auto &i : dists) {
        for (auto &j : i) {
            sum += j.second;
        }        
    }
    return sum;
}

void Graph::FindCriticalEdge(double k) {
    //Edge* critical = nullptr;
    int criticalLeft = -1;
    int criticalRight = -1;
    int size = edges.size();
    int count = 0;
    double distances = 0;
    if (k >= 0.0 && k <= 1.0) {
        double min = DBL_MAX;
        for (auto &v : edges) {
            cout << count++ << " out of " << size << endl;
            for (auto &edge : v.second) {
                double PrevWeight = edge.GetWeight();
                edges[edge.GetRight()].erase(Edge(0, edge.GetLeft(), 0));
                edges[edge.GetRight()].insert(Edge(edge.GetRight(), edge.GetLeft(), PrevWeight*k));
                ((Edge&)edge).SetWeight(PrevWeight*k);
                double sum = RunDijkstraAsync();
                if (min > sum) {
                    min = sum;
                    criticalLeft = edge.GetLeft();
                    criticalRight = edge.GetRight();
                    //critical = &((Edge&)edge);
                }
                ((Edge&)edge).SetWeight(PrevWeight);
                edges[edge.GetRight()].erase(Edge(0, edge.GetLeft(), 0));
                edges[edge.GetRight()].insert(Edge(edge.GetRight(), edge.GetLeft(), PrevWeight));
            }
        }
        distances = min;
    }
    else if (k >= 1.0) {
        double max = 0;
        for (auto &v : edges) {
            cout << count++ << " out of " << size << endl;
            for (auto &edge : v.second) {
                double PrevWeight = edge.GetWeight();
                edges[edge.GetRight()].erase(Edge(0, edge.GetLeft(), 0));
                edges[edge.GetRight()].insert(Edge(edge.GetRight(), edge.GetLeft(), PrevWeight*k));
                ((Edge&)edge).SetWeight(PrevWeight*k);
                double sum = RunDijkstraAsync();
                if (max < sum) {
                    max = sum;
                    criticalLeft = edge.GetLeft();
                    criticalRight = edge.GetRight();
                }
                ((Edge&)edge).SetWeight(PrevWeight);
                edges[edge.GetRight()].erase(Edge(0, edge.GetLeft(), 0));
                edges[edge.GetRight()].insert(Edge(edge.GetRight(), edge.GetLeft(), PrevWeight));
            }
        }
        distances = max;
    }
    else {

    }
    if (criticalLeft != -1 && criticalRight != -1) {
        auto edge = edges[criticalLeft].find(Edge(0, criticalRight, 0));
        cout << "New distances: " << distances << ". With edge between ";
        cout << coord_to_vertecies[edge->GetLeft()] << " and " << coord_to_vertecies[edge->GetRight()] << ". Weight: " << edge->GetWeight() << endl;
    }
}

//Private methods

void Graph::Dijkstra(int v) {
    //std::vector<double> dist(edges.size(), inf);
    std::vector<bool> visited(edges.size(), false);
    //std::vector<int> path(edges.size(), -1);
    dists[v][v] = 0.0;
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, Compare> q;
    visited[v] = true;
    q.push(std::make_pair(dists[v][v], v));

    while (!q.empty()) {
        std::pair<double, int> from = q.top(); q.pop();
        for (auto& edge : edges[from.second]) {
            int to = edge.GetRight();
            if (!visited[to]) {
                double tmp = from.first + edge.GetWeight();
                auto it = dists[v].find(to);
                if (it == dists[v].end() || it->second > tmp) {
                    dists[v][to] = tmp;
                    visited[to] = false;
                    q.push(std::make_pair(dists[v][to], to));
                }
            }
        }
    }
}

std::vector<double> Graph::oldDijksra(int v) {
    std::vector<double> dist(edges.size(), inf);
    std::vector<bool> visited(edges.size(), false);
    //std::vector<int> path(edges.size(), -1);
    dist[v] = 0.0;
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, Compare> q;
    visited[v] = true;
    q.push(std::make_pair(dists[v][v], v));

    while (!q.empty()) {
        std::pair<double, int> from = q.top(); q.pop();
        for (auto& edge : edges[from.second]) {
            int to = edge.GetRight();
            if (!visited[to]) {
                double tmp = from.first + edge.GetWeight();
                if (dist[to] > tmp) {
                    dist[to] = tmp;
                    visited[to] = false;
                    //path[to] = from.second;
                    q.push(std::make_pair(dist[to], to));
                }
            }
        }
    }
    return dist;
}

void Graph::RunDijkstraThread(int from, int len) {
    for (int i = from; i < from + len; ++i)
        Dijkstra(i);
}

void Graph::OldRunDijkstraThread(int from, int len) {
    for (int i = from; i < from + len; ++i)
        oldDists[i] =  oldDijksra(i);
}

void Graph::AddEdge(int index, int vertex, double weight) {
    edges[index].insert(Edge(index, vertex, weight));
}
