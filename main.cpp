#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>

#include "graph.h"
#include "treedecomposition.h"

#include "LKH.h"

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
int mainWrapperTest(vector<string> args) {
    // Convert the args string vector to a c style char array array
    int argc = args.size();
    if (argc <= 0)
        return 1337;
    vector<shared_ptr<char>> argv;
    for (unsigned int i=0; i<args.size(); ++i) {
        shared_ptr<char> pStr = shared_ptr<char>(new char[args[i].size() + 1]);
        for (unsigned int j=0; j<args[i].size() + 1; ++j) {
            pStr.get()[j] = args[i].c_str()[j];
        }
        argv.push_back(pStr);
    }

    // Use the C style char array array
    //return mainWrapper(argc, &argv[0]);
    //return test(40);
    return 42;
}

int main()
{
    // Test KLH C code calls
    vector<string> lkhArgs = { "pr2392.par" };
    int result = mainWrapperTest(lkhArgs);
    cout << result << endl;

    // Test if the from file and to file functions work - and compute a new tree decomposition
    Graph* pG = new Graph();
    unique_ptr<TreeDecomposition> pTD = unique_ptr<TreeDecomposition>(new TreeDecomposition(pG));

    graphsFromFile(*pG, *pTD, "test-graph-in.txt");
    cout << "Done graph from file" << endl;

    pTD->MinimumDegree();
    cout << "Done minimum degree heuristic" << endl;

    graphsToFile(*pG, *pTD, "test-graph-out.txt");
    cout << "Done graph to file" << endl;

    // Hi there world
    cout << endl << "Hello world!" << endl;
    return 0;
}
