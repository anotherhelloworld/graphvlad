#include "graph.h"

using namespace std;

int main(int argc, char* argv[]) {
    //Graph graph(argv[1]);
    Graph graph("vlad-2009.dat");
    //Graph graph("vl-grand.dat");
    //graph.ParseLinksRegEx("vl-grand/links.txt");
    cout << "Original distances: " << graph.RunDijkstraAsync() << endl;
    graph.FindCriticalEdge(0.5);
    //graph.RunDijkstraAsync();    
    //graph.Print();
    //graph.ParseLinks("links.txt");
    return 0;
}