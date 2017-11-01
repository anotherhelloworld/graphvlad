#include "edge.h"

void Edge::IncTravelCount() {
    ++travelCount;
}

double Edge::GetTotalWeight() {
    return travelCount * weight;
}


bool operator==(const Edge& e, const Edge& t) {
    return e.GetRight() == t.GetRight();
}