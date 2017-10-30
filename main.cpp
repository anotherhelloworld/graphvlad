#include "graph.h"
using namespace std;

int main(int argc, char* argv[]) {
    //Graph graph(argv[1]);
    Graph graph("");
    //graph.Print();
    graph.ParseLinks("links.txt");
    return 0;
}