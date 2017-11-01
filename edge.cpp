#include "edge.h"

void Edge::incTravelCount() {
    ++travelCount;
}

double Edge::getTotalWeight() {
    return travelCount * weight;
}
