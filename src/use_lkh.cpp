#include "use_lkh.h"

#include <iostream>
#include <fstream>
#include <memory>

#include "graph.h"
#include "utils.h"

// The header for the renamed main and run functions (for LKH)
extern "C" {
    #include "LKH.h"
    int main_lkh(int argc, char *argv[]);
}

//
//  The functions
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

    // Use the C style string array to call the LKH main function
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

pair<int, vector<int>> readTourFile(string path, int startVid) {
    // Read tour integers from file, add them to the graph and return the length of the found tour (or -1 if it didn't add any edges).
    ifstream in(path);
    bool inTourSection = false; // Make sure it is an actual tour file, that doesn't have integers lying around not part of the tour...
    int tourLength = 0;
    vector<int> vids;
    for (string line; getline(in, line); ) {
        line = trim(line);
        if (comp(line, "COMMENT : Length = ")) {
            string substring = line.substr(string("COMMENT : Length = ").size());
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
        vids.push_back(stoi(line) - startVid);
    }
    // Complete the cycle
    int test = vids.size();
    vids.push_back(vids[0]);
    return make_pair(tourLength, vids);
}

void generateLKHFiles(const Graph& graph, const vector<int>& tourVids, string tempDir, string name, int runs) {
    // The paths for the lkh files
    string problemFile = tempDir + name + ".tsp";
    string tourFile = tempDir + name + ".tour";
    string parameterFile = tempDir + name + ".par";

    // Create a string with the graph info in it
    int startVid = 1;
    string s = "DIMENSION : " + to_string(tourVids.size()) + "\n";
    s += "NAME : TEMP_LKH\n";
    s += "TYPE : TSP\n";
    s += "EDGE_WEIGHT_TYPE : EUC_2D\n";

    s += "NODE_COORD_SECTION\n";
    for (int i=0; i<tourVids.size(); ++i) {
        Vertex* pV = graph.vertices[tourVids[i]].get();
        s += to_string(i + startVid) + " " + to_string(pV->x) + " " + to_string(pV->y) + "\n"; // Note the 'wrong' vids
    }

    // Write the graph to file
    ofstream out(problemFile);
    out << s;
    out.close();

    // Write the parameter file
    ofstream out2(parameterFile);
    out2 << "PROBLEM_FILE = " + problemFile + "\nMOVE_TYPE = 5\nPATCHING_C = 3\nPATCHING_A = 2\nRUNS = " + to_string(runs) + "\nTOUR_FILE = " + tourFile + "\nTRACE_LEVEL = 0\n";
    out2.close();

    // Write the (empty) tour file
    ofstream out3(tourFile);
    out3 << " ";
    out3.close();
}

pair<int, vector<int>> lkh_tsp(const Graph& graph, const vector<int>& tourVids, int runs /*=3*/) {
    // Return the cost vertices (in order) of this TSP tour as calculated by LKH.

    // Special case
    if (tourVids.size() == 2)
        return make_pair(2 * calculateEuclidean(graph.vertices[tourVids[0]].get(), graph.vertices[tourVids[1]].get()), vector<int>(tourVids));

    // Some constants
    string tempDir = "vrp-files/temp/";
    string name = "temp";

    // Solve the TSP on a subset of tour vids
    string parameterFile = tempDir + name + ".par";
    string tourFile = tempDir + name + ".tour";
    generateLKHFiles(graph, tourVids, tempDir, name, runs);
    mainWrapper({ "", parameterFile });

    // Read from file
    int startVid = 1;
    pair<int, vector<int>> result = readTourFile(tourFile, startVid);
    vector<int> vids;
    for (int i=0; i<result.second.size(); ++i)
        vids.push_back(tourVids[result.second[i]]); // Correct the 'wrong' vids

    return make_pair(result.first, vids);
}
