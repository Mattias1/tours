#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>

#include "graph.h"
#include "treedecomposition.h"
#include "utils.h"
#include "dp_tsp.h"
using namespace std;

// The header for the renamed main and run functions ~Matty
extern "C" {
    #include "LKH.h"
    int main_lkh(int argc, char *argv[]);
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

int tourFromFile(Graph& rGraph, string path) {
    // Read tour integers from file, add them to the graph and return the length of the found tour (or -1 if it didn't add any edges).
    ifstream in(path);
    bool inTourSection = false; // Make sure it is an actual tour file, that doesn't have integers lying around not part of the tour...
    int tourLength = 0;
    vector<unsigned int> vids;
    for (string line; getline(in, line); ) {
        line = trim(line);
        if (comp(line, "COMMENT : Length = ")) {
            string substring = line.substr(strlen("COMMENT : Length = "));
            if (isInt(substring))
                tourLength = stoi(substring);
            continue;
        }
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
    vids.push_back(vids[0]); // This seems to be the problem...
    // Now let the graph add its edges
    if (rGraph.AddTourFromFile(vids))
        return tourLength;
    return -1;
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
//  Some wrappers around LKH functions
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

void runWrapper() {
    // A wrapper for an LKH run after it's been initialized in the main function
    // Not the most beautiful of solutions, but this hopefully will stop the program from randomly crashing
    // All code in here is copy pasted from somewhere in main_lkh, and then modified a bit

    // Find the tour and it's cost
    int cost = FindTour();

    // Record the tour (write to file, I hope)
    RecordBetterTour();
    RecordBestTour();
    WriteTour(OutputTourFileName, BestTour, cost);
    WriteTour(TourFileName, BestTour, cost);
}

//
// The main function
//
int main()
{
    // The main entrypoint for this application. Here we load the graph (they should only contain vertices),
    // then run LKH, merge the tours, create a tree decomposition and finally calculate the optimal tour on this decomposition using DP.

    // The names of the tsp-problems we want to solve
    vector<string> FILES = { "mod502" };
    // The number of LKH runs
    int LKH_RUNS = 5;

    // Get the tour for each of the files
    for (unsigned int i=0; i<FILES.size(); ++i) {
        string file = "tsp-files/" + FILES[i];
        string tempFile = "tsp-files/temp/" + FILES[i];

        // Load the graph
        Graph* pG = new Graph(); // pTD will be the owner of pG.
        unique_ptr<TreeDecomposition> pTD = unique_ptr<TreeDecomposition>(new TreeDecomposition(pG));
        graphsFromFile(*pG, *pTD, file + ".tsp");
        cout << "Done graph from file" << endl << "----------------------------" << endl;

        // Run LKH and merge the tours
        int mergedTours = 1;
        vector<string> lkhArgs = { "", file + ".par" }; // The first argument is the programs name, though the empty string should be fine.
        mainWrapper(lkhArgs);
        int tourLength = tourFromFile(*pG, tempFile + ".tour");
        if (tourLength == -1) {
            cout << "ERROR: The first tour is not added, whut?" << endl;
            return 1;
        }
        graphsToFile(*pG, *pTD, tempFile + "_0.txt");
        cout << "Added first tour (" << tempFile << "_0.txt - " << tourLength << ")" << endl;
        for (int r=1; r<LKH_RUNS; ++r) {
            runWrapper();
            tourLength = tourFromFile(*pG, tempFile + ".tour");
            if (tourLength != -1) {
                graphsToFile(*pG, *pTD, tempFile + "_" + to_string(r) + ".txt");
                cout << "Added new tour   (" << tempFile << "_" << r << ".txt - " << tourLength << ")" << endl;
                ++mergedTours;
            }
            else {
                cout << "LKH found identical tour" << endl;
            }
        }
        cout << "----------------------------" << endl << "Done LKH; merged " << mergedTours << " tours" << endl;

        // Create the tree decomposition
        pTD->MinimumDegree();
        int treewidth = pTD->GetTreeWidth();
        cout << "Done minimum degree heuristic (treewidth: " << treewidth << ")" << endl;

        // Write the final graph with decomposition to file
        graphsToFile(*pG, *pTD, tempFile + "_final.txt");
        cout << "Done final graph to file" << endl;

        // Run the DP
        if (treewidth > 10) {
            cout << "Treewidth too large for the DP; aborting program." << endl << "----------------------------" << endl;
            continue;
        }
        tspDP(*pTD);
        cout << "Done DP for TSP" << endl << "----------------------------" << endl;
    }
    return 0;
}
