#include "edge.h"

bool operator==(const Edge& e, const Edge& t) {
    return e.GetRight() == t.GetRight();
}