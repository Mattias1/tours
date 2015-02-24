#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>

#include "graph.h"
#include "treedecomposition.h"

#include "LKH.h"
#include "test.h"

using namespace std;

// The header for the renamed main function (and a test function) ~Matty
extern "C" {
    int main_lkh(int argc, char *argv[]);
    int test(int nr);
}

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
int mainWrapper(vector<string> args) {
    // Convert the args string vector to a c style char array array
    int argc = args.size();
    if (argc <= 0)
        return 1337;
    vector<unique_ptr<char>> argvMemoryManager;
    vector<char*> argv;
    for (unsigned int i=0; i<args.size(); ++i) {
        unique_ptr<char> pStr = unique_ptr<char>(new char[args[i].size() + 1]);
        for (unsigned int j=0; j<args[i].size() + 1; ++j) {
            pStr.get()[j] = args[i].c_str()[j];
        }
        argv.push_back(pStr.get());
        argvMemoryManager.push_back(move(pStr));
    }

    // Use the C style char array array
    return main_lkh(argc, &argv[0]);
    // return test(40);
}

int main()
{
    // Test KLH C code calls
    vector<string> lkhArgs = { "", "tsp-files/pr2392.par" }; // The first argument is the programs name, though the empty string should be fine.
    int result = mainWrapper(lkhArgs);
    cout << "LKH main result: " << result << endl;

    // Test if the from file and to file functions work - and compute a new tree decomposition
    Graph* pG = new Graph();
    unique_ptr<TreeDecomposition> pTD = unique_ptr<TreeDecomposition>(new TreeDecomposition(pG));

    graphsFromFile(*pG, *pTD, "test-graph-in.txt");
    cout << "Done graph from file" << endl;

    pTD->MinimumDegree();
    cout << "Done minimum degree heuristic" << endl;

    graphsToFile(*pG, *pTD, "test-graph-out.txt");
    cout << "Done graph to file" << endl;
    return 0;
}
