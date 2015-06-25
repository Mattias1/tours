#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>

#include "graph.h"
#include "treedecomposition.h"
#include "utils.h"
#include "dp_tsp.h"
#include "dp_vrp.h"
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
    vector<int> vids;
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

void graphsToFile(const Graph& graph, string path) {
    // 'Convert' the graph to string
    string s = graph.ToFileString();
    // Write the graphs to file (apparently this ignores all errors)
    ofstream out(path);
    out << s;
    out.close();
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
        return 1;
    vector<unique_ptr<char>> argvMemoryManager;
    vector<char*> argv;
    for (int i=0; i<args.size(); ++i) {
        unique_ptr<char> pStr = unique_ptr<char>(new char[args[i].size() + 1]);
        for (int j=0; j<args[i].size() + 1; ++j) {
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
bool unitTests() {
    // Run some unit tests
    bool debug = false;

    bool testResult = true;
    if (debug)
        cout << "Unit tests:\n------------------------" << endl;

    // Distribute demands 1:
    // void distributeDemands(vector<vector<int>>& rResult, vector<int>& rLoop, int demandLeft, int sizeLeft) {
    // Find all permutations of demands (or capacities, w/e - int's with min value 2) for a single path and store them in the result array.
    // So for a given demand of 6 and a size of 2, this will add [4,2], [3,3] and [2,4] to the rResult list (rLoop initialized as vector of size 2).
    vector<vector<int>> result;
    vector<int> loop = vector<int>(2, 0);
    distributeDemands(result, loop, 6, 2);
    if (debug)
        cout << "distribute demands 1: " << dbg(result) << endl;
    if (dbg(result) != "[6,0 - 5,1 - 4,2 - 3,3 - 2,4 - 1,5 - 0,6]")
        testResult = false;

    // Finish
    cout << "Unit tests completed.\n------------------------" << endl;
    return testResult;
}

void debug() {
    // Load the graph (including tree decomposition)
    Graph* pG = new Graph(); // pTD will be the owner of pG.
    unique_ptr<TreeDecomposition> pTD = unique_ptr<TreeDecomposition>(new TreeDecomposition(pG));
    graphsFromFile(*pG, *pTD, "test-graph.txt");
    int treewidth = pTD->GetTreeWidth();
    pTD->CreateRoot();
    cout << "Done graph from file - treewidth: " << treewidth << endl << "----------------------------" << endl;

    // RUN TSP
    vector<Edge*> tourEdges = tspDP(*pTD);
    cout << "Done DP for TSP" << endl;
}

int runTSP(vector<string> FILES, int LKH_RUNS) {
    // Calculate the tour for each of the files
    cout << "Running TSP experiments." << endl;
    for (int i=0; i<FILES.size(); ++i) {
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
        graphsToFile(*pG, tempFile + "_0.txt");
        cout << "Added first tour (" << tempFile << "_0.txt - " << tourLength << ")" << endl;
        for (int r=1; r<LKH_RUNS; ++r) {
            runWrapper();
            tourLength = tourFromFile(*pG, tempFile + ".tour");
            if (tourLength != -1) {
                graphsToFile(*pG, tempFile + "_" + to_string(r) + ".txt");
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

        // Write the merged graph of LKH tours with its decomposition to file
        graphsToFile(*pG, *pTD, tempFile + "_merged.txt");
        cout << "Done merged graph to file" << endl;

        // Run the DP
        if (treewidth > 10) {
            cout << "Treewidth too large for the DP; aborting run." << endl << "----------------------------" << endl;
            continue;
        }
        vector<Edge*> tourEdges = tspDP(*pTD);
        cout << "Done DP for TSP" << endl;

        // Write the result graph with decomposition to file
        unique_ptr<Graph> pResultingTourGraph = pG->CreateTourGraph(tourEdges);
        graphsToFile(*pResultingTourGraph, tempFile + "_result.txt");
        cout << "Done result graph to file" << endl << "----------------------------" << endl;
    }
    return 0;
}

int runVRP(vector<string> FILES, int SAVINGS_RUNS, int SWEEP_RUNS) {
    // Calculate a set of routes for each of the files
    cout << "Running VRP experiments." << endl;
    for (int i=0; i<FILES.size(); ++i) {
        string file = "vrp-files/" + FILES[i];
        string tempFile = "vrp-files/temp/" + FILES[i];

        // Load the graph
        Graph* pG = new Graph(); // pTD will be the owner of pG.
        unique_ptr<TreeDecomposition> pTD = unique_ptr<TreeDecomposition>(new TreeDecomposition(pG));
        graphsFromFile(*pG, *pTD, file + ".vrp");
        if (pG->trucks == 0 || pG->capacity == 0) {
            cout << "ERROR in file " << file << ".vrp: TRUCKS = " << pG->trucks << " and CAPACITY = " << pG->capacity << endl;
            continue;
        }
        cout << "Done graph from file" << endl << "----------------------------" << endl;

        // Run heuristics and merge the tours
        int mergedTours = 1;
        // vector<string> lkhArgs = { "", file + ".par" }; // The first argument is the programs name, though the empty string should be fine.
        // mainWrapper(lkhArgs);
        // int tourLength = tourFromFile(*pG, tempFile + ".tour");
        // if (tourLength == -1) {
        //     cout << "ERROR: The first tour is not added, whut?" << endl;
        //     return 1;
        // }
        // graphsToFile(*pG, tempFile + "_0.txt");
        // cout << "Added first tour (" << tempFile << "_0.txt - " << tourLength << ")" << endl;
        // for (int r=1; r<LKH_RUNS; ++r) {
        //     runWrapper();
        //     tourLength = tourFromFile(*pG, tempFile + ".tour");
        //     if (tourLength != -1) {
        //         graphsToFile(*pG, tempFile + "_" + to_string(r) + ".txt");
        //         cout << "Added new tour   (" << tempFile << "_" << r << ".txt - " << tourLength << ")" << endl;
        //         ++mergedTours;
        //     }
        //     else {
        //         cout << "LKH found identical tour" << endl;
        //     }
        // }
        // cout << "----------------------------" << endl << "Done LKH; merged " << mergedTours << " tours" << endl;

        // Create the tree decomposition
        // pTD->MinimumDegree();
        pTD->CreateRoot(); // Remove this once the Minimum degree is called - it roots the tree for us.
        int treewidth = pTD->GetTreeWidth();
        cout << "Done minimum degree heuristic (treewidth: " << treewidth << ")" << endl;

        // Write the merged graph of LKH tours with its decomposition to file
        graphsToFile(*pG, *pTD, tempFile + "_merged.txt");
        cout << "Done merged graph to file" << endl;

        // Run the DP
        if (treewidth > 10) {
            cout << "Treewidth too large for the DP; aborting run." << endl << "----------------------------" << endl;
            continue;
        }
        vector<vector<Edge*>> tourEdges = vrpDP(*pTD);
        cout << "Done DP for VRP" << endl;

        // Write the result graph with decomposition to file
        unique_ptr<Graph> pResultingTourGraph = pG->CreateTourGraph(tourEdges);
        graphsToFile(*pResultingTourGraph, tempFile + "_result.txt");
        cout << "Done result graph to file" << endl << "----------------------------" << endl;
    }
    return 0;
}

int main(int argc, char *argv[])
{
    // The main entry-point for this application. Here we load the graph (they should only contain vertices),
    // then run LKH, merge the tours, create a tree decomposition and finally calculate the optimal tour on this decomposition using DP.
    vector<string> args(argv, argv + argc); // Currently not used
    bool TSP = false;

    // DEBUG
    assert(unitTests());

    // Run TSP algorithms
    if (TSP) {
        vector<string> FILES = { "mod502" };
        int LKH_RUNS = 5;

        return runTSP(FILES, LKH_RUNS);
    }
    // Run VRP algorithms
    else {
        vector<string> FILES = { "test-vrp" };
        int SAVINGS_RUNS = 1;
        int SWEEP_RUNS = 0;

        return runVRP(FILES, SAVINGS_RUNS, SWEEP_RUNS);
    }
}

// RANDOM IDEA: a possible optimization might be to fill sub-tables if not all demand is used... somewhere...
// TODO: Make sure the depot vertex is in every bag
// TODO: save in a bag the total demand of all vertices in that bag and below, so that we can return early in bad cases
