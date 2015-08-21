#define _USE_MATH_DEFINES

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <assert.h>

#include "graph.h"
#include "treedecomposition.h"
#include "utils.h"
#include "dp_tsp.h"
#include "dp_vrp.h"
#include "use_lkh.h"
#include "savings.h"
#include "sweep.h"
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

int tourFromFile(Graph& rGraph, string path) {
    // Read the tour file
    int startVid = 1;
    pair<int, vector<int>> result = readTourFile(path, startVid);
    int tourLength = result.first;
    vector<int> vids = result.second;

    // Add the edges to the graph
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
// The main function
//
bool unitTests() {
    // Run some unit tests
    bool debug = false;

    bool testResult = true;
    if (debug)
        cout << "Unit tests:\n------------------------" << endl;


    // Distribute demands 1:
    vector<vector<vector<int>>> allSubPathDemands = vector<vector<vector<int>>>(1);
    vector<int> loop = vector<int>(2, 0);
    distributeDemands(allSubPathDemands[0], loop, 6, 2);
    if (debug)
        cout << "distribute demands 1: " << dbg(allSubPathDemands[0]) << endl;
    if (dbg(allSubPathDemands[0]) != "[6,0 - 5,1 - 4,2 - 3,3 - 2,4 - 1,5 - 0,6]")
        testResult = false;

    // Find all child matchings 1:
    // MatchingEdge main1 = MatchingEdge(1, 2, 6);
    MatchingEdge sub1 = MatchingEdge(1, 3, -1);
    MatchingEdge sub2 = MatchingEdge(2, 3, -1);
    vector<vector<MatchingEdge*>> childEndpoints = {{ &sub1 }, { &sub2 }};
    vector<vector<pair<int, int>>> pathList = {{ make_pair(0, 0), make_pair(1, 0) }};
    vector<vector<vector<MatchingEdge>>> result = vector<vector<vector<MatchingEdge>>>(childEndpoints.size());
    vector<vector<MatchingEdge>> loop2 = vector<vector<MatchingEdge>>(childEndpoints.size());
    for (int j=0; j<loop2.size(); ++j)
        loop2[j] = vector<MatchingEdge>(childEndpoints[j].size());
    fillAllChildMatchings(result, loop2, 0, childEndpoints, pathList, allSubPathDemands);
    if (debug) {
        cout << "all child matchings 1: " << endl;
        for (int j=0; j<result.size(); ++j)
            cout << "  " << j << ": " << dbg(result[j]) << endl;
        cout << endl;
    }
    if (result.size() != 2 || dbg(result[0]) != "[1-3:6 - 1-3:5 - 1-3:4 - 1-3:3 - 1-3:2 - 1-3:1 - 1-3:0]"
                           || dbg(result[1]) != "[2-3:0 - 2-3:1 - 2-3:2 - 2-3:3 - 2-3:4 - 2-3:5 - 2-3:6]") {
        testResult = false;
    }

    // Distribute demands 2:
    allSubPathDemands = vector<vector<vector<int>>>(2);
    vector<int>pathDemands = {3, 1};
    pathList = {{ make_pair(0, 0) }, {}};
    for (int i=0; i<pathDemands.size(); ++i) {
        if (pathList[i].size() > 0) {
            loop = vector<int>(pathList[i].size(), 0);
            distributeDemands(allSubPathDemands[i], loop, pathDemands[i], pathList[i].size());
        }
    }
    if (debug) {
        cout << "distribute demands 2: " << endl;
        for (int i=0; i<allSubPathDemands.size(); ++i)
            cout << "    " << i << ": " << dbg(allSubPathDemands[i]) << ", size: " << allSubPathDemands[i].size() << endl;
    }
    if (allSubPathDemands.size() != 2 || dbg(allSubPathDemands[0]) != "[3]" || dbg(allSubPathDemands[1]) != "[]" || allSubPathDemands[1].size() != 0)
        testResult = false;

    // Find all child matchings 2:
    sub1 = MatchingEdge(0, 0, -1);
    childEndpoints = {{ &sub1 }};
    pathList = {{ make_pair(0, 0)}, {}};
    result = vector<vector<vector<MatchingEdge>>>(childEndpoints.size());
    loop2 = vector<vector<MatchingEdge>>(childEndpoints.size());
    for (int j=0; j<loop2.size(); ++j)
        loop2[j] = vector<MatchingEdge>(childEndpoints[j].size());
    fillAllChildMatchings(result, loop2, 0, childEndpoints, pathList, allSubPathDemands);
    if (debug) {
        cout << "all child matchings 2: " << endl;
        for (int j=0; j<result.size(); ++j)
            cout << "  " << j << ": " << dbg(result[j]) << endl;
        cout << endl;
    }
    if (result.size() != 1 || dbg(result[0]) != "[0-0:3]")
        testResult = false;

    // Find all child matchings 3:
    allSubPathDemands = {{ {2,0}, {1,1}, {0,2} },   { {3,0}, {2,1}, {1,2}, {0,3} }};
    sub1 = MatchingEdge(0, 0, -1);
    sub2 = MatchingEdge(2, 3, -1);
    MatchingEdge sub3 = MatchingEdge(0, 4, -1);
    MatchingEdge sub4 = MatchingEdge(3, 4, -1);
    childEndpoints = {{ &sub1, &sub2, &sub3 }, { &sub4 }};
    pathList = {{ make_pair(0, 0), make_pair(0, 1)}, { make_pair(0, 2), make_pair(1, 0) }};
    result = vector<vector<vector<MatchingEdge>>>(childEndpoints.size());
    loop2 = vector<vector<MatchingEdge>>(childEndpoints.size());
    for (int j=0; j<loop2.size(); ++j)
        loop2[j] = vector<MatchingEdge>(childEndpoints[j].size());
    fillAllChildMatchings(result, loop2, 0, childEndpoints, pathList, allSubPathDemands);
    if (debug) {
        cout << "all child matchings 3: " << endl;
        for (int j=0; j<result.size(); ++j)
            cout << "  " << j << ": " << dbg(result[j]) << endl;
        cout << endl;
    }
    if (result.size() != 2
            || dbg(result[0]) != "[0-0:2,2-3:0,0-4:3 - 0-0:2,2-3:0,0-4:2 - 0-0:2,2-3:0,0-4:1 - 0-0:2,2-3:0,0-4:0 - 0-0:1,2-3:1,0-4:3 - 0-0:1,2-3:1,0-4:2 - 0-0:1,2-3:1,0-4:1 - 0-0:1,2-3:1,0-4:0 - 0-0:0,2-3:2,0-4:3 - 0-0:0,2-3:2,0-4:2 - 0-0:0,2-3:2,0-4:1 - 0-0:0,2-3:2,0-4:0]"
            || dbg(result[1]) != "[3-4:0 - 3-4:1 - 3-4:2 - 3-4:3 - 3-4:0 - 3-4:1 - 3-4:2 - 3-4:3 - 3-4:0 - 3-4:1 - 3-4:2 - 3-4:3]") {
        testResult = false;
    }

    // childEndpoints:
    //     vector, size = nr of bags
    //       matchings-list, size = nr of matchings (or paths) for this child
    // result:
    //     vector, size = nr of child bags (+1 for parent bag)
    //       vector, size = nr of combinations (I don't know the amount, but it increases every time)
    //         matchings-list, size = nr of matchings (or paths) for this child
    // pathList:
    //     vector, size = nr of paths
    //       vector, size = nr of subpaths for this path
    //         pair: (child-index, matching-index (in the child's matching list))
    // allSubPathDemands:
    //     vector, size = nr of paths
    //       vector, size = nr of possibilities
    //         vector, size = nr of subpaths for this path

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
        cout << "Done graph from file (" << FILES[i] << ")" << endl << "----------------------------" << endl;

        // Run LKH and merge the tours
        int mergedTours = 1;
        vector<string> lkhArgs = { "", file + ".par" }; // The first argument is the programs name, though the empty string should be fine.
        mainWrapper(lkhArgs);
        int tourLength = tourFromFile(*pG, tempFile + ".tour");
        assert(tourLength != -1 && "The first tour is not added, whut?");
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
                cout << "LKH found no new edges" << endl;
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
        if (tourEdges.size() > 0) {
            unique_ptr<Graph> pResultingTourGraph = pG->CreateTourGraph(tourEdges);
            graphsToFile(*pResultingTourGraph, tempFile + "_result.txt");
            cout << "Done result graph to file" << endl;
        }
        cout << "----------------------------" << endl;
    }
    return 0;
}

int runVRP(vector<string> FILES, int SAVINGS_RUNS, int SWEEP_RUNS) {
    // Calculate a set of routes for each of the files
    cout << "Running VRP experiments." << endl;
    for (int i=0; i<FILES.size(); ++i) {
        if (i > 0) {
            cout << "Currently we can only manage 1 tour per run. Just start again with the next file please." << endl;
            return 0;
        }
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
        cout << "Done graph from file (" << FILES[i] << ")" << endl << "----------------------------" << endl;

        // Run heuristics and merge the tours
        int mergedTours = 0;
        int tourLength = -1;
        if (SAVINGS_RUNS > 0) {
            tourLength = savings(*pG);
            if (tourLength != -1) {
                graphsToFile(*pG, tempFile + "_0.txt");
                cout << "  Added saving tours (" << tempFile << "_0.txt - " << tourLength << ")" << endl;
                ++mergedTours;
            }
            else {
                cout << "  Savings could not find a solution" << endl; // This is possible if TRUCKS is very strict (or just too small)
            }
        }
        for (int r=1; r<=SWEEP_RUNS; ++r) {
            tourLength = sweep(*pG);
            if (tourLength != -1) {
                graphsToFile(*pG, tempFile + "_" + to_string(r) + ".txt");
                cout << "  Added sweep tours  (" << tempFile << "_" << r << ".txt - " << tourLength << ")" << endl;
                ++mergedTours;
            }
            else {
                cout << "  Sweep found no new edges" << endl;
            }
        }
        cout << "----------------------------" << endl << "Done heuristics; merged " << mergedTours << " tours" << endl;

        // Create the tree decomposition
        pTD->MinimumDegree(true);
        // pTD->CreateRoot(); // Remove this once the Minimum degree is called - it roots the tree for us.
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
        if (tourEdges.size() > 0) {
            unique_ptr<Graph> pResultingTourGraph = pG->CreateTourGraph(tourEdges);
            graphsToFile(*pResultingTourGraph, tempFile + "_result.txt");
            cout << "Done result graph to file" << endl;
        }
        cout << "----------------------------" << endl;
    }
    return 0;
}

int main(int argc, char *argv[])
{
    // The main entry-point for this application. Here we load the graph (they should only contain vertices),
    // then run LKH, merge the tours, create a tree decomposition and finally calculate the optimal tour on this decomposition using DP.
    vector<string> args(argv, argv + argc); // Currently not used
    bool TSP = true;

    // DEBUG
    assert(unitTests());

    // Run TSP algorithms
    if (TSP) {
        vector<string> FILES = { "mod502" };
        int LKH_RUNS = 35;

        return runTSP(FILES, LKH_RUNS);
    }
    // Run VRP algorithms
    else {
        vector<string> FILES = { "full-vrp-1" };
        int SAVINGS_RUNS = 1;
        int SWEEP_RUNS = 4;

        return runVRP(FILES, SAVINGS_RUNS, SWEEP_RUNS);
    }
}

// RANDOM IDEA: a possible optimization might be to fill sub-tables if not all demand is used... somewhere...
// TODO: save in a bag the total demand of all vertices in that bag and below, so that we can return early in bad cases
//       (well, this might not work. What if vertices also appear above, and are used (partially?) above)
// TODO: 'exactly k' or 'at most k' trucks? (savings does 'at most k' right now, and algorithm does exaclty k - it doesn't matter really)

// TODO: check demands in vrp edge select (!)
