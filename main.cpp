#include "graph.h"
using namespace std;

int main(int argc, char* argv[]) {
    Graph graph(argv[1]);
    graph.Print();
    return 0;
}