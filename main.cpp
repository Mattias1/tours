#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>

#include "graph.h"
#include "treedecomposition.h"
#include "utils.h"
using namespace std;

// The header for the renamed main function (and a test function) ~Matty
extern "C" int main_lkh(int argc, char *argv[]);

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

bool tourFromFile(Graph& rGraph, string path) {
    // Read tour integers from file
    ifstream in(path);
    bool inTourSection = false; // Make sure it is an actual tour file, that doesn't have integers lying around not part of the tour...
    vector<unsigned int> vids;
    for (string line; getline(in, line); ) {
        // Loop and ignore everything until I reach the tour section (and make sure I stay in there)
        if (comp(line, "TOUR_SECTION")) {
            inTourSection = true;
            continue;
        }
        if (!inTourSection) {
            continue;
        }
        if (!isInt(line)) {
            if (line != "-1")
                cout << "ERROR IN TOUR FILE - line: " << line << " - (main.cpp - tourFromFile)" << endl;
            break;
        }
        // We are in the tour section part
        vids.push_back(stoi(line));
    }
    // Complete the cycle
    vids.push_back(vids[0]);
    // Now let the graph add its edges
    return rGraph.AddTourFromFile(vids);
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
    // The wrapper for the (renamed) main function of LKH.
    // Convert the args string vector to an array of c style strings (array of char arrays)
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

    // Use the C style string array
    return main_lkh(argc, &argv[0]);
}

int main()
{
    // The main entrypoint for this application. Here we load the graph (they should only contain vertices),
    // then run LKH, merge the tours, create a tree decomposition and finally calculate the optimal tour on this decomposition using DP.

    // The names of the tsp-problems we want to solve
    vector<string> FILES = { "pr2392" };
    // The number of LKH runs
    int LKH_RUNS = 5;

    // Get the tour for each of the files
    for (unsigned int i=0; i<FILES.size(); ++i) {
        string file = "tsp-files/" + FILES[i];

        // Load the graph
        Graph* pG = new Graph();
        unique_ptr<TreeDecomposition> pTD = unique_ptr<TreeDecomposition>(new TreeDecomposition(pG));

        graphsFromFile(*pG, *pTD, file + ".tsp");
        cout << "Done graph from file" << endl << "----------------------------" << endl;

        // Run LKH and merge tours
        int mergedTours = 0;
        vector<string> lkhArgs = { "", file + ".par" }; // The first argument is the programs name, though the empty string should be fine.
        for (int r=0; r<LKH_RUNS; ++r) {
            mainWrapper(lkhArgs);
            if (tourFromFile(*pG, file + ".tour")) {
                cout << "Added new tour" << endl;
                ++mergedTours;
            }
            else {
                cout << "LKH found identical tour" << endl;
            }
        }
        cout << "----------------------------" << endl << "Done LKH; merged " << mergedTours << " tours" << endl;

        // Create the tree decomposition
        pTD->MinimumDegree();
        cout << "Done minimum degree heuristic (treewidth: " << pTD->GetTreeWidth() << ")" << endl;

        // Run the DP

        // Write a graph to file.... not sure why I want to do that, but ok???
        graphsToFile(*pG, *pTD, "test-graph-out.txt");
        cout << "Done graph to file" << endl << "----------------------------" << endl << endl;
    }
    return 0;
}
