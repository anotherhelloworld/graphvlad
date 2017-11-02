#include "graph.h"

using namespace std;

int main(int argc, char* argv[]) {
    //Graph graph(argv[1]);
    Graph graph("vlad-2009.dat");
    //graph.RunDijkstraAsync();
    graph.ParseLinksRegEx("vl-grand\\links.txt");
    //graph.Print();
    //graph.ParseLinks("links.txt");
    return 0;
}