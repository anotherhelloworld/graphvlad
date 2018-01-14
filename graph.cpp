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
    //dists.resize(edges.size());
    
    //for (int i = 0; i < edges.size(); ++i)
    //    Dijkstra(i);
    //oldDists.resize(edges.size());
    distSum.resize(edges.size());
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
    for (auto &i : distSum)
        sum += i;       
    return sum;
}

void Graph::FindCriticalEdge(double k) {
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

double Graph::Dijkstra(int v) {
    std::vector<double> dist(edges.size(), inf);
    std::vector<bool> visited(edges.size(), false);
    dist[v] = 0.0;
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, Compare> q;
    visited[v] = true;
    q.push(std::make_pair(dist[v], v));

    while (!q.empty()) {
        std::pair<double, int> from = q.top(); q.pop();
        for (auto& edge : edges[from.second]) {
            int to = edge.GetRight();
            if (!visited[to]) {
                double tmp = from.first + edge.GetWeight();
                if (dist[to] > tmp) {
                    dist[to] = tmp;
                    visited[to] = true;
                    q.push(std::make_pair(dist[to], to));
                }
            }
        }
    }
    double sum = 0;
    for (auto &i : dist)
        if (i != inf)
            sum += i;
    return sum;
}

void Graph::RunDijkstraThread(int from, int len) {
    for (int i = from; i < from + len; ++i)
        distSum[i] = Dijkstra(i);
}

void Graph::AddEdge(int index, int vertex, double weight) {
    edges[index].insert(Edge(index, vertex, weight));
}
