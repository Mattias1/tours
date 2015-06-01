#ifndef DP_VRP_H
#define DP_VRP_H

#include "dp_tsp.h"
#include "graph.h"
#include "treedecomposition.h"
#include <vector>
#include <string>
#include <unordered_map>
using namespace std;

//
//  The core functions used to calculate the tour for the VRP problem.
//
vector<vector<Edge*>> vrpDP(const TreeDecomposition& TD, bool consoleOutput = true);

int vrpTable(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const string& S, const Bag& Xi);
vector<vector<Edge*>> vrpReconstruct(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const string& S, const Bag& Xi);

int vrpChildEvaluation(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge*>& Yi, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<Matching*>& rEndpoints, vector<vector<Matching*>>& rChildEndpoints);
vector<Edge*>* vrpLookback(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge*>& Yi, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<Matching*>& rEndpoints, vector<vector<Matching*>>& rChildEndpoints);

int vrpRecurse(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge*>& Yi, int i, int j, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<Matching*>& rEndpoints, vector<vector<Matching*>>& rChildEndpoints);
vector<vector<Edge*>> vrpRecurseVector(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge*>& Yi, int i, int j, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<Matching*>& rEndpoints, vector<vector<Matching*>>& rChildEndpoints);

vector<pair<int, vector<Matching>>> vrpEdgeSelect(int cost, int minimum, int index, const Graph& graph, const Bag& Xi, const vector<Edge*>& Yi, const vector<int>& degrees, vector<Matching*>& rEndpoints, vector<Matching*>& rAllChildEndpoints, int edgeListBits = 0);

//
// Some helper functions
//
bool isDepot(Vertex* pV);

vector<Matching> pathDemands(const Graph& graph, const Bag& Xi, const vector<Edge*>& edgeList, const vector<Matching*>& endpoints, const vector<Matching*>& allChildEndpoints);

vector<vector<vector<Matching>>> allChildEndpointPossibilities(const Graph& graph, const Bag& Xi, const vector<Edge*>& Yi, const vector<Edge*>& edgeList, const vector<Matching*>& endpoints, const vector<vector<Matching*>>& childEndpoints, const vector<Matching>& edgeDemands);

void distributeDemands(vector<vector<int>>& rResult, vector<int>& rLoop, int demandLeft, int size);

void fillAllChildMatchings(vector<vector<vector<Matching>>>& rResult, vector<vector<Matching>>& rLoop, int pathIndex, const vector<vector<Matching*>>& childEndpoints, const vector<vector<pair<int, int>>>& pathList, const vector<vector<vector<int>>>& allSubPathDemands);

#endif // DP_VRP_H
