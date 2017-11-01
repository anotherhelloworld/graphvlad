#pragma once

class Edge {
public:
    Edge(int u, int v, double w) : left(u), right(v), weight(w) {}
    void incTravelCount();
    double getTotalWeight();
    const int getLeft() { return left; }
    const int getRight() { return right; }
    const double getWeight() { return weight; }
    void setWeight(double w) { weight = w; }
private:
    int left;
    int right;
    double weight;
    int travelCount;
};
