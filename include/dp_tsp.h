#ifndef DP_TSP_H
#define DP_TSP_H

#include "graph.h"
#include "treedecomposition.h"
#include <vector>
#include <string>
#include <unordered_map>
using namespace std;

//
// The matching class
//
class MatchingEdge
{
    public:
        int A, B, Demand;

        MatchingEdge();
        MatchingEdge(int a, int b);
        MatchingEdge(int a, int b, int demand);

        int Other(int vid) const;

        bool IsIncidentTo(int vid) const;
        bool EqualsSortOf(int vid1, int vid2) const;

        static bool MergeInto(int a, int b, vector<MatchingEdge*>& rMatchings, vector<unique_ptr<MatchingEdge>>& rNewMatchingsMemoryManager, bool dontMergeDepot = false);

    private:
        int* getPointerTo(int vid);
};

ostream &operator<<(ostream &os, const MatchingEdge& m);

//
//  The core functions used to calculate the tour for the TSP problem.
//
vector<Edge*> tspDP(const TreeDecomposition& TD, bool consoleOutput = true);

int tspTable(const Graph& graph, unordered_map<string, int>& rHashlist, const string& S, const Bag& Xi);
vector<Edge*> tspReconstruct(const Graph& graph, unordered_map<string, int>& rHashlist, const string& S, const Bag& Xi);

int tspRecurse(const Graph& graph, unordered_map<string, int>& rHashlist, const Bag& Xi, const vector<Edge*>& Yi, int i, int j, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<MatchingEdge*>& rEndpoints, vector<vector<MatchingEdge*>>& rChildEndpoints);
vector<Edge*> tspRecurseVector(const Graph& graph, unordered_map<string, int>& rHashlist, const Bag& Xi, const vector<Edge*>& Yi, int i, int j, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<MatchingEdge*>& rEndpoints, vector<vector<MatchingEdge*>>& rChildEndpoints);

int tspChildEvaluation(const Graph& graph, unordered_map<string, int>& rHashlist, const Bag& Xi, const vector<Edge*>& Yi, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<MatchingEdge*>& rEndpoints, const vector<vector<MatchingEdge*>>& rChildEndpoints, vector<Edge*>* pResultingEdgeList = nullptr);
vector<Edge*> tspLookback(const Graph& graph, unordered_map<string, int>& rHashlist, const Bag& Xi, const vector<Edge*>& Yi, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<MatchingEdge*>& rEndpoints, vector<vector<MatchingEdge*>>& rChildEndpoints);

int tspEdgeSelect(int minimum, int index, const Graph& graph, const Bag& Xi, const vector<Edge*>& Yi, const vector<int>& degrees, vector<MatchingEdge*>& rEndpoints, vector<MatchingEdge*>& rAllChildEndpoints, int* pEdgeListBits = nullptr);

//
// Some helper functions - also used in the VRP DP
//
vector<int> tableEntryToDegrees(const string& S);
vector<MatchingEdge> tableEntryToEndpoints(const string& S);
string toTableEntry(const Bag& Xi, const vector<int>& degrees, const vector<MatchingEdge*>& endpoints);
string toTableEntry(const Bag& Xi, const vector<int>& degrees, const vector<MatchingEdge>& endpoints);

bool cycleCheck(const Graph& graph, const vector<MatchingEdge*>& endpoints, vector<Edge*>* pEdgeList, vector<MatchingEdge*>& rAllChildEndpoints);

bool inEndpoints(const vector<MatchingEdge*>& endpoints, int vid1, int vid2);

vector<Edge*> removeDoubles(const vector<Edge*>& edges, int length);

int toEdgeListBits(const vector<Edge*>& Yi, const vector<Edge*>& resultingEdgeList);
void addToEdgeListFromBits(const vector<Edge*>& Yi, vector<Edge*>* pResultingEdgeList, int edgeListBits);

#endif // DP_TSP_H
