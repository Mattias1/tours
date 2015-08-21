#include "savings.h"

#include <list>
#include <algorithm>
#include <assert.h>
#include <iostream>

#include "use_lkh.h"
#include "utils.h"

//
// The saving class
//
Saving::Saving(int i, int j, const Graph& graph, vector<int> costDepot2i)
    :I(i),
    J(j),
    Value(costDepot2i[i] + costDepot2i[j] - calculateEuclidean(graph.vertices[i].get(), graph.vertices[j].get()))
{ }

ostream &operator<<(ostream &os, const Saving& s) {
    return os << s.I << "," << s.J << ": " << s.Value;
}

//
// The algorithm
//
inline bool concatenatePaths(int i, int j, bool frontI, bool frontJ, vector<pair<int, list<int>>*>& rPaths) {
    // Helper function for the savings function below - unsurprisingly (I hope), it concatenates paths
    if (!frontI && frontJ) {
        // Update the i-th linked list
        rPaths[i]->first += rPaths[j]->first;
        rPaths[i]->second.splice(rPaths[i]->second.end(), rPaths[j]->second);
        // The paths that are no longer front or back should be null and the new front and back should be set properly
        pair<int, list<int>>* pPath = rPaths[i];
        rPaths[i] = nullptr;
        rPaths[j] = nullptr;
        rPaths[pPath->second.front()] = pPath;
        rPaths[pPath->second.back()] = pPath;
        return true;
    }
    else if (frontI && !frontJ) {
        // Update the j-th linked list
        rPaths[j]->first += rPaths[i]->first;
        rPaths[j]->second.splice(rPaths[j]->second.end(), rPaths[i]->second);
        // The paths that are no longer front or back should be null and the new front and back should be set properly
        pair<int, list<int>>* pPath = rPaths[j];
        rPaths[i] = nullptr;
        rPaths[j] = nullptr;
        rPaths[pPath->second.front()] = pPath;
        rPaths[pPath->second.back()] = pPath;
        return true;
    }
    return false;
}

int savings(Graph& rGraph) {
    // Calculate a set of tours using the savings heuristic, and add the edges to the graph
    bool debug = false;

    // Precompute all edge costs (0, i)
    vector<int> costDepot2i = vector<int>(rGraph.vertices.size());
    for (int i=0; i<costDepot2i.size(); ++i)
        costDepot2i[i] = calculateEuclidean(rGraph.vertices[0].get(), rGraph.vertices[i].get());

    // Calculate and sort all savings (not for the depot obviously)
    vector<Saving> savings;
    for (int i=1; i<rGraph.vertices.size() - 1; ++i) {
        for (int j=i+1; j<rGraph.vertices.size(); ++j)
            savings.push_back(Saving(i, j, rGraph, costDepot2i));
    }
    auto sortLambda = [&](const Saving& a, const Saving& b) {
        return a.Value > b.Value;
    };
    sort(savings.begin(), savings.end(), sortLambda);
    if (debug) {
        cout << "Savings: " << endl;
        for (int s=0; s<savings.size(); ++s) {
            cout << "  " << savings[s] << endl;
        }
    }

    // Initialize all tours (way too many of them) (we don't skip the depot for easy, direct integer access, but it is ignored in the code)
    vector<pair<int, list<int>>> pathsMemoryManager = vector<pair<int, list<int>>>(rGraph.vertices.size()); // Memory manager of all the seperate paths
    vector<pair<int, list<int>>*> paths;                                                                    // Each index i points to the path that contains the i-th vertex as start or end, otherwise null
    for (int i=0; i<rGraph.vertices.size(); ++i) {
        pathsMemoryManager[i] = make_pair(rGraph.vertices[i]->demand, list<int>(1, i));
        paths.push_back(&pathsMemoryManager[i]);
    }

    // Merge the paths (best feasible merge)
    for (int s=0; s<savings.size(); ++s) {
        // Test if both I and J appear at the front or back of a path
        int i = savings[s].I;
        int j = savings[s].J;
        if (paths[i] == nullptr || paths[j] == nullptr)
            continue;
        bool frontI, frontJ;
        if (paths[i]->second.back() == i)
            frontI = false;
        else if (paths[i]->second.front() == i)
            frontI = true;
        else
            assert(false && "ERROR (savings): In the case that the i-th vid is not a front or back, this pointer should be null");
        if (paths[j]->second.front() == j)
            frontJ = true;
        else if (paths[j]->second.back() == j)
            frontJ = false;
        else
            assert(false && "ERROR (savings): In the case that the j-th vid is not a front or back, this pointer should be null");
        // Make sure the paths don't exceed the capacity of a truck
        if (paths[i]->first + paths[j]->first > rGraph.capacity) {
            continue;
        }

        // So now we know that the current saving makes for a feasible merge, go merge it
        if (!concatenatePaths(i, j, frontI, frontJ, paths)) {
            if (paths[i]->second.size() <= paths[j]->second.size()) {
                paths[i]->second.reverse();
                frontI = !frontI;
            }
            else {
                paths[j]->second.reverse();
                frontJ = !frontJ;
            }
            if (!concatenatePaths(i, j, frontI, frontJ, paths)) {
                assert(false && "ERROR (savings): Path concatenation failed twice");
            }
        }
        if (debug) {
            cout << "s: {" << savings[s] << "}, paths:" << endl << dbg(paths) << endl;
        }
    }

    // Determine the tour-edges (strictly speaking: the order of the vertices) for the graph (skip the depot again)
    vector<vector<int>> vids;
    for (int i=1; i<paths.size(); ++i) {
        // Test if this tour isn't processed already
        if (paths[i] == nullptr || paths[i]->second.front() < i || paths[i]->second.back() < i)
            continue;
        // Collect the vertex ids
        vids.push_back({ 0 });
        for (auto iterVid = paths[i]->second.begin(); iterVid != paths[i]->second.end(); iterVid++) {
            vids.back().push_back(*iterVid);
        }
        vids.back().push_back(0);
    }

    // Add the tour-edges to the graph
    if (vids.size() > rGraph.trucks) {
        cout << "Savings - Too many trucks used: " << vids.size() << endl;
        return -1;
    }
    int totalValue = 0;
    bool somethingNew = false;
    for (int i=0; i<vids.size(); ++i) {
        // Optimize the tour using LKH
        pair<int, vector<int>> result = lkh_tsp(rGraph, vids[i]);
        if (result.first == -1)
            return -1;
        totalValue += result.first;
        if (rGraph.AddTourFromFile(result.second))
            somethingNew = true;
    }

    if (somethingNew)
        return totalValue;
    else
        return -1;
}
