#include "edge.h"

void Edge::IncTravelCount() {
    ++travelCount;
}

double Edge::GetTotalWeight() {
    return travelCount * weight;
}
