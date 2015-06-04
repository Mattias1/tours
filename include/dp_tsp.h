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
class Matching
{
    public:
        int A, B, Demand;

        Matching();
        Matching(int a, int b);
        Matching(int a, int b, int demand);

        int Other(int vid) const;

        bool IsIncidentTo(int vid) const;
        bool EqualsSortOf(int vid1, int vid2) const;

        static bool MergeInto(int a, int b, vector<Matching*>& rMatchings, vector<Matching>& rNewMatchingsMemoryManager);

    private:
        int* getPointerTo(int vid);
};

ostream &operator<<(ostream &os, const Matching& m);

//
//  The core functions used to calculate the tour for the TSP problem.
//
vector<Edge*> tspDP(const TreeDecomposition& TD, bool consoleOutput = true);

int tspTable(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const string& S, const Bag& Xi);
vector<Edge*> tspReconstruct(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const string& S, const Bag& Xi);

int tspChildEvaluation(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge*>& Yi, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<Matching*>& rEndpoints, const vector<vector<Matching*>>& rChildEndpoints, vector<Edge*>* pResultingEdgeList = nullptr);
vector<Edge*> tspLookback(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge*>& Yi, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<Matching*>& rEndpoints, vector<vector<Matching*>>& rChildEndpoints);

int tspRecurse(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge*>& Yi, int i, int j, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<Matching*>& rEndpoints, vector<vector<Matching*>>& rChildEndpoints);
vector<Edge*> tspRecurseVector(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge*>& Yi, int i, int j, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<Matching*>& rEndpoints, vector<vector<Matching*>>& rChildEndpoints);

int tspEdgeSelect(int minimum, int index, const Graph& graph, const Bag& Xi, const vector<Edge*>& Yi, const vector<int>& degrees, vector<Matching*>& rEndpoints, vector<Matching*>& rAllChildEndpoints, int* pEdgeListBits = nullptr);

//
// Some helper functions - also used in the VRP DP
//
vector<int> toDegrees(const string& S);
vector<Matching> toEndpoints(const string& S);
string fromDegreesEndpoints(const vector<int>& degrees, const vector<Matching*>& endpoints);

bool cycleCheck(const Graph& graph, const vector<Matching*>& endpoints, vector<Edge*>* pEdgeList, vector<Matching*>& rAllChildEndpoints);

bool inEndpoints(const vector<Matching*>& endpoints, int vid1, int vid2);

vector<Edge*> removeDoubles(const vector<Edge*>& edges, int length);

int toEdgeListBits(const vector<Edge*>& Yi, const vector<Edge*>& resultingEdgeList);
void addToEdgeListFromBits(const vector<Edge*>& Yi, vector<Edge*>* pResultingEdgeList, int edgeListBits);

#endif // DP_TSP_H
