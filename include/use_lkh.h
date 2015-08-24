#ifndef USE_LKH_H
#define USE_LKH_H

#include <string>
#include <vector>

#include "graph.h"
using namespace std;

//
//  The functions
//
int mainWrapper(vector<string> args);

void runWrapper();

pair<int, vector<int>> readTourFile(string path, int startVid);

void generateLKHFiles(const Graph& graph, const vector<int>& tourVids, string tempDir, string name, int runs);

pair<int, vector<int>> lkh_tsp(const Graph& graph, const vector<int>& tourVids, int runs = 10);

#endif // USE_LKH_H
