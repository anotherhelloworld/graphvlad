#include "graph.h"

using namespace std;

int main(int argc, char* argv[]) {
    cout << "Reading graph" << endl;
    Graph graph;
    if (argc > 1)
        graph.open(argv[1]);
    else
        graph.open("little.dat");
    cout << "Read graph successfully" << endl;
    cout << "Counting original distances" << endl;
    cout << "Original distances: " << graph.RunDijkstraAsync() << endl;
    double k = 1.5;
    if (argc > 2)
        k = atof(argv[2]);
    cout << "Searching critical edge" << endl;
    graph.FindCriticalEdge(k);
    return 0;
}
