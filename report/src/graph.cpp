
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
        int id = 0;
        
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
                  

                  
                  AddEdge(u, v, weight, id++);
                  AddEdge(v, u, weight, id++);
        } 
        

        
        for (auto &i : coord)
            coord_to_vertecies[i.second] = i.first;
        
    }
    distSum.resize(edges.size(), 0);
} 


double Graph::RunDijkstraAsync() {
    
    int threads_count = 8;
    

    
    std::vector<thread> threads;
    

    int batch = edges.size() / threads_count;
    int remainder = edges.size() % threads_count;

    
    for (int i = 0; i < threads_count - 1; ++i)
            threads.push_back(thread(
              &Graph::RunDijkstraThread, 
              this, 
              i * batch, 
              batch
            ));
        threads.push_back(thread(
            &Graph::RunDijkstraThread, 
            this, 
            (threads_count - 1) * batch, 
            batch + remainder
          ));

      for (int i = 0; i < threads_count; ++i)
            threads[i].join(); 
    
    
    
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
          cout << count++ << " out of " << size << flush;
          for (auto &edge : v.second) {
              if (usedEdges.find(edge.getId()) == usedEdges.end()) {
                  double PrevWeight = edge.GetWeight();
                  auto it = edges[edge.GetRight()]
                    .find(Edge(0, edge.GetLeft(), 0, 0));
                  int invId = it->getId();
                  usedEdges.insert(invId);
                  edges[edge.GetRight()].erase(it);
                  edges[edge.GetRight()]
                    .insert(Edge(
                      edge.GetRight(), 
                      edge.GetLeft(), 
                      PrevWeight*k, 
                      invId));
                  ((Edge&)edge).SetWeight(PrevWeight*k);
                  double sum = RunDijkstraAsync();
                  if (min > sum) {
                      min = sum;
                      criticalLeft = edge.GetLeft();
                      criticalRight = edge.GetRight();
                  }
                  ((Edge&)edge).SetWeight(PrevWeight);
                  edges[edge.GetRight()]
                    .erase(Edge(0, edge.GetLeft(), 0, 0));
                  edges[edge.GetRight()]
                    .insert(Edge(
                      edge.GetRight(), 
                      edge.GetLeft(), 
                      PrevWeight, 
                      invId));
              }
          }
          cout << "\r";
      }
      cout << endl;
      distances = min;
      
    }
    else if (k >= 1.0) {
      
      double max = 0;
      for (auto &v : edges) {
          cout << count++ << " out of " << size << flush;
          for (auto &edge : v.second) {
              if (usedEdges.find(edge.getId()) == usedEdges.end()) {
                  double PrevWeight = edge.GetWeight();
                  auto it = edges[edge.GetRight()]
                    .find(Edge(0, edge.GetLeft(), 0, 0));
                  int invId = it->getId();
                  usedEdges.insert(invId);
                  edges[edge.GetRight()].erase(it);
                  edges[edge.GetRight()]
                    .insert(Edge(
                      edge.GetRight(), 
                      edge.GetLeft(), 
                      PrevWeight*k, 
                      invId));
                  ((Edge&)edge).SetWeight(PrevWeight*k);
                  double sum = RunDijkstraAsync();
                  if (max < sum) {
                      max = sum;
                      criticalLeft = edge.GetLeft();
                      criticalRight = edge.GetRight();
                  }
                  ((Edge&)edge).SetWeight(PrevWeight);
                  edges[edge.GetRight()]
                    .erase(Edge(0, edge.GetLeft(), 0, 0));
                  edges[edge.GetRight()]
                    .insert(Edge(
                      edge.GetRight(), 
                      edge.GetLeft(), 
                      PrevWeight, 
                      invId));
              }
          }
          cout << "\r";
      }
      cout << endl;
      distances = max;
        
    }

    
    if (criticalLeft != -1 && criticalRight != -1) {
        auto edge = edges[criticalLeft].find(Edge(0, criticalRight, 0, 0));
        cout << "New distances: " << distances << ". With edge between ";
        cout << coord_to_vertecies[edge->GetLeft()] 
          << " and " 
          << coord_to_vertecies[edge->GetRight()] 
          << ". Weight: " 
          << edge->GetWeight() << endl;
    } 
    
}


double Graph::Dijkstra(int v) {

    
    std::vector<double> dist(edges.size(), inf);
    std::vector<bool> visited(edges.size(), false);

    dist[v] = 0.0;
    std::priority_queue<
      std::pair<double, int>, 
      std::vector<std::pair<double, int>>, 
      Compare> q;
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
                        q.push(std::make_pair(tmp, to));
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


void Graph::AddEdge(int index, int vertex, double weight, int id) {
    edges[index].insert(Edge(index, vertex, weight, id));
}

