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

int vrpChildEvaluation(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge*>& edges, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<int>& rEndpoints, vector<vector<int>>& rChildEndpoints, vector<Edge*>* pResultingEdgeList = nullptr);
vector<Edge*>* vrpLookback(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge>& edges, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<int>& rEndpoints, vector<vector<int>>& rChildEndpoints);

int vrpRecurse(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge*>& edges, unsigned int i, unsigned int j, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<int>& rEndpoints, vector<vector<int>>& rChildEndpoints);
vector<vector<Edge*>> vrpRecurseVector(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge*>& edges, unsigned int i, unsigned int j, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<int>& rEndpoints, vector<vector<int>>& rChildEndpoints);

vector<pair<int, vector<int>>> vrpEdgeSelect(int cost, int minimum, unsigned int index, const Graph& graph, const Bag& Xi, const vector<Edge*>& edges, const vector<int>& degrees, vector<int>& rEndpoints, vector<int>& rAllChildEndpoints, vector<Edge*>* pEdgeList = nullptr);

//
// Some helper functions
//
bool isDepot(Vertex* pV);

vector<int> pathDemands(const Graph& graph, const Bag& Xi, const vector<Edge*>& edgeList, const vector<int>& endpoints, const vector<int>& allChildEndpoints);

#endif // DP_VRP_H
