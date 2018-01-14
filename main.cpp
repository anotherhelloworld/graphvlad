#include "graph.h"

using namespace std;

int main(int argc, char* argv[]) {
    cout << "Reading graph" << endl;
    Graph graph;
    if (argc > 1)
        graph.open(argv[1]);
    else
        graph.open("vlad-2009.dat");
    cout << "Read graph successfully" << endl;
    //Graph graph("vl-grand.dat");
    //graph.ParseLinksRegEx("vl-grand/links.txt");
    cout << "Counting original distances" << endl;
    cout << "Original distances: " << graph.RunDijkstraAsync() << endl;
    double k = 0.5;
    if (argc > 2)
        k = atoi(argv[2]);
    cout << "Searching critical edge" << endl;
    graph.FindCriticalEdge(k);
    //graph.RunDijkstraAsync();    
    //graph.Print();
    //graph.ParseLinks("links.txt");
    return 0;
}
