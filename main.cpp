#include "graph.h"

using namespace std;

int main(int argc, char* argv[]) {
    Graph graph;
    if (argc > 1)
        graph.open(argv[1]);
    else
        graph.open("vlad-2009.dat");
    //Graph graph("vl-grand.dat");
    //graph.ParseLinksRegEx("vl-grand/links.txt");
    cout << "Original distances: " << graph.RunDijkstraAsync() << endl;
    double k = 0.5;
    if (argc > 2)
        k = atoi(argv[2]);
    graph.FindCriticalEdge(k);
    //graph.RunDijkstraAsync();    
    //graph.Print();
    //graph.ParseLinks("links.txt");
    return 0;
}
