#pragma once

class Edge {
public:
    Edge(int u, int v, double w) : left(u), right(v), weight(w) {}
    void incTravelCount();
    void IncTravelCount();
    double GetTotalWeight();
    const int GetLeft() const { return left; }
    const int GetRight() const { return right; }
    const double GetWeight() const { return weight; }
    void SetWeight(double w) { weight = w; }
private:
    int left;
    int right;
    double weight;
    int travelCount;
};
