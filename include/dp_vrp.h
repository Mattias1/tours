#ifndef DP_VRP_H
#define DP_VRP_H

#include "dp_tsp.h"
#include "graph.h"
#include "treedecomposition.h"
#include <vector>
#include <string>
#include <unordered_map>
#include <tuple>
using namespace std;

//
//  The core functions used to calculate the tour for the VRP problem.
//
vector<vector<Edge*>> vrpDP(const TreeDecomposition& TD, bool consoleOutput = true);

int vrpTable(const Graph& graph, unordered_map<string, int>& rHashlist, const string& S, const Bag& Xi);
vector<vector<Edge*>> vrpReconstruct(const Graph& graph, unordered_map<string, int>& rHashlist, const string& S, const Bag& Xi);

int vrpRecurse(const Graph& graph, unordered_map<string, int>& rHashlist, const Bag& Xi, const vector<Edge*>& Yi, int i, int j, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<MatchingEdge*>& rEndpoints, vector<vector<MatchingEdge*>>& rChildEndpoints, bool ignoreD2D = false);
vector<vector<Edge*>> vrpRecurseVector(const Graph& graph, unordered_map<string, int>& rHashlist, const Bag& Xi, const vector<Edge*>& Yi, int i, int j, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<MatchingEdge*>& rEndpoints, vector<vector<MatchingEdge*>>& rChildEndpoints);

int vrpChildEvaluation(const Graph& graph, unordered_map<string, int>& rHashlist, const Bag& Xi, const vector<Edge*>& Yi, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<MatchingEdge*>& rEndpoints, vector<vector<MatchingEdge*>>& rChildEndpoints);
vector<Edge*>* vrpLookback(const Graph& graph, unordered_map<string, int>& rHashlist, const Bag& Xi, const vector<Edge*>& Yi, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<MatchingEdge*>& rEndpoints, vector<vector<MatchingEdge*>>& rChildEndpoints);

vector<tuple<int, int, vector<MatchingEdge>>> vrpEdgeSelect(int cost, int minimum, int index, const Graph& graph, const Bag& Xi, const vector<Edge*>& Yi, const vector<int>& degrees, vector<MatchingEdge*>& rEndpoints, vector<MatchingEdge*>& rAllChildEndpoints, int edgeListBits = 0);

vector<MatchingEdge> pathDemands(const Graph& graph, const Bag& Xi, const vector<Edge*>& edgeList, const vector<MatchingEdge*>& endpoints, const vector<MatchingEdge*>& allChildEndpoints);

vector<vector<vector<MatchingEdge>>> allChildMatchings(const Graph& graph, const Bag& Xi, const vector<Edge*>& Yi, const vector<Edge*>& edgeList, const vector<MatchingEdge*>& endpoints, const vector<vector<MatchingEdge*>>& childEndpoints);

void distributeDemands(vector<vector<int>>& rResult, vector<int>& rLoop, int demandLeft, int size);

void fillAllChildMatchings(vector<vector<vector<MatchingEdge>>>& rResult, vector<vector<MatchingEdge>>& rLoop, int pathIndex, const vector<vector<MatchingEdge*>>& childEndpoints, const vector<vector<pair<int, int>>>& pathList, const vector<vector<vector<int>>>& allSubPathDemands);

int singleCityTourSpecialCaseManager(const Graph& graph, vector<int>& rDegrees);

//
// Some helper functions
//
inline bool isDepot(int vid) {
    return vid == 0;
}
inline bool isDepot(Vertex* pV) {
    return isDepot(pV->vid);
}

#endif // DP_VRP_H
