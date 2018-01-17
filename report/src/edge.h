
#pragma once
#include <atomic>
#include <functional>

class Edge {
public:
    
    Edge(const Edge& e) :
        left(e.left), 
        right(e.right), 
        weight(e.weight), 
        id(e.id) { }
    Edge(int u, int v, double w, int id) : 
        left(u), 
        right(v), 
        weight(w), 
        id(id) { }
    
    
    const int GetLeft() const { return left; }
    const int GetRight() const { return right; }
    const double GetWeight() const { return weight; }
    const int getId() const { return id; }
    void SetWeight(double w) { weight = w; }
    
private:
    int left;
    int right;
    double weight;
    int id;
};


struct EdgeHash {
    unsigned int operator()(const Edge& e) const {
        return std::hash<int>()(e.GetRight());
    }
};

bool operator==(const Edge& e, const Edge& t);


