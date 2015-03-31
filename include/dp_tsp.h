#ifndef DP_TSP_H
#define DP_TSP_H

#include "graph.h"
#include "treedecomposition.h"
#include <vector>
#include <string>
#include <unordered_map>
using namespace std;

//
//  All sorts of functions used to calculate the tour for the TSP problem.
//
vector<Edge*> tspDP(const TreeDecomposition& TD);

int tspTable(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const string& S, const Bag& Xi);
vector<Edge*> tspReconstruct(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const string& S, const Bag& Xi);

int tspChildEvaluation(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge*>& edges, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<int>& rEndpoints, vector<vector<int>>& rChildEndpoints, vector<Edge*>* pResultingEdgeList = nullptr);
vector<Edge*>* tspLookback(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge>& edges, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<int>& rEndpoints, vector<vector<int>>& rChildEndpoints);

int tspRecurse(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge*>& edges, unsigned int i, unsigned int j, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<int>& rEndpoints, vector<vector<int>>& rChildEndpoints);
vector<Edge*> tspRecurseVector(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge*>& edges, unsigned int i, unsigned int j, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<int>& rEndpoints, vector<vector<int>>& rChildEndpoints);

int tspEdgeSelect(int minimum, unsigned int index, const Graph& graph, const Bag& Xi, const vector<Edge*>& edges, const vector<int>& degrees, vector<int>& rEndpoints, vector<int>& rAllChildEndpoints, vector<Edge*>* pEdgeList = nullptr);

vector<int> toDegrees(const string& S);
vector<int> toEndpoints(const string& S);
string fromDegreesEndpoints(const vector<int>& degrees, const vector<int>& endpoints);

bool cycleCheck(const Graph& graph, vector<int>& rEndpoints, vector<Edge*>* pEdgeList, vector<int>& rAllChildEndpoints);

bool inEndpoints(const vector<int>& endpoints, int startVid, int endVid);

#endif // DP_TSP_H