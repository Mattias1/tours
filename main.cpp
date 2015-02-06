#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "graph.h"
#include "treedecomposition.h"

#include <algorithm>

using namespace std;

//
//  File I/O
//
void graphsFromFile(Graph& rGraph, TreeDecomposition& rTreeDecomposition, string path) {
    // Read the graphs from file
    ifstream in(path);
    int state = 0;
    for (string line; getline(in, line); ) {
        // Handle the line management
        if (not rGraph.ReadFileLine(state, line))
            rTreeDecomposition.ReadFileLine(state, line);
    }
}

void graphsToFile(const Graph& graph, const TreeDecomposition& treeDecomposition, string path) {
    // 'Convert' the graph to string
    string s = graph.ToFileString();
    s += treeDecomposition.ToFileString();
    // Write the graphs to file (apparently this ignores all errors)
    ofstream out(path);
    out << s;
    out.close();
}

//
//  The main function
//
int main()
{
    // Test if the from file and to file functions work - and compute a new tree decomposition
    shared_ptr<Graph> pG = shared_ptr<Graph>(new Graph());
    shared_ptr<TreeDecomposition> pTD = shared_ptr<TreeDecomposition>(new TreeDecomposition(pG));
    Graph& rG = *pG;
    TreeDecomposition& rTD = *pTD;

    graphsFromFile(rG, rTD, "test-graph-in.txt");

    //pTD = TreeDecomposition::MinimumDegree(pG);

    graphsToFile(rG, rTD, "test-graph-out.txt");

    // Hi there world
    cout << "Hello world!" << endl;
    return 0;
}
