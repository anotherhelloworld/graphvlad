#pragma once
#include <atomic>
#include <functional>

class Edge {
public:
    Edge(const Edge& e) :left(e.left), right(e.right), weight(e.weight) {} //todo check how to fix travelCount(e.travelCount)
    Edge(int u, int v, double w) : left(u), right(v), weight(w) {}
    const int GetLeft() const { return left; }
    const int GetRight() const { return right; }
    const double GetWeight() const { return weight; }
    void SetWeight(double w) { weight = w; }
private:
    int left;
    int right;
    double weight;
};

struct EdgeHash {
    unsigned int operator()(const Edge& e) const {
        return std::hash<int>()(e.GetRight());
    }
};

bool operator==(const Edge& e, const Edge& t);
