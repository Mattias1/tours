#include "savings.h"

#include <list>
#include <algorithm>
#include <assert.h>

//
// The saving class
//
Saving::Saving(int i, int j, const Graph& graph, vector<int> costDepot2i)
    :I(i),
    J(j),
    Value(costDepot2i[i] + costDepot2i[j] + calculateEuclidean(graph.vertices[i].get(), graph.vertices[j].get()))
{ }

//
// The algorithm
//
inline bool concatenatePaths(int i, int j, bool frontI, bool frontJ, vector<pair<int, list<int>>*>& rPaths) {
    // Helper function for the savings function below - unsurprisingly (I hope), it concatenates paths
    if (!frontI && frontJ) {
        rPaths[i]->first += rPaths[j]->first;
        rPaths[i]->second.splice(rPaths[i]->second.end(), rPaths[j]->second);
        rPaths[j] = rPaths[i]; // j is in the i-th path now
        return true;
    }
    else if (frontI && !frontJ) {
        rPaths[j]->first += rPaths[i]->first;
        rPaths[j]->second.splice(rPaths[j]->second.end(), rPaths[i]->second);
        rPaths[i] = rPaths[j];
        return true;
    }
    return false;
}

int savings(Graph& rGraph) {
    // Precompute all edge costs (0, i)
    Vertex* pDepot = rGraph.vertices[0].get();
    vector<int> costDepot2i = vector<int>(rGraph.vertices.size());
    for (int i=0; i<costDepot2i.size(); ++i)
        costDepot2i[i] = calculateEuclidean(pDepot, rGraph.vertices[i].get());

    // Calculate and sort all savings
    vector<Saving> savings;
    for (int i=0; i<rGraph.vertices.size() - 1; ++i) {
        for (int j=i+1; j<rGraph.vertices.size(); ++j)
            savings.push_back(Saving(i, j, rGraph, costDepot2i));
    }
    auto sortLambda = [&](const Saving& a, const Saving& b) {
        return a.Value > b.Value;
    };
    sort(savings.begin(), savings.end(), sortLambda);

    // Initialize all tours (way too many of them)
    vector<pair<int, list<int>>> pathsMemoryManager;    // Memory manager of all the seperate paths
    vector<pair<int, list<int>>*> paths;                // Each index i points to the path that contains the i-th vertex
    for (int i=0; i<rGraph.vertices.size(); ++i) {
        pathsMemoryManager.push_back(make_pair(rGraph.vertices[i]->demand, list<int>(1, i)));
        paths.push_back(&pathsMemoryManager[i]);
    }

    // Merge the paths (best feasible merge)
    for (int s=0; s<savings.size(); ++s) {
        // Test if both I and J appear at the front or back of a path
        int i = savings[s].I;
        int j = savings[s].J;
        bool frontI, frontJ;
        if (paths[i]->second.back() == i)
            frontI = false;
        else if (paths[i]->second.front() == i)
            frontI = true;
        else if (paths[j]->second.front() == j)
            frontJ = true;
        else if (paths[j]->second.back() == j)
            frontJ = false;
        else
            continue;
        // Make sure the paths don't exceed the capacity of a truck
        if (paths[i]->first + paths[j]->first > rGraph.capacity)
            continue;
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
    }

    // Determine the tour-edges (strictly speaking: the order of the vertices) for the graph
    vector<vector<int>> vids;
    for (int i=0; i<paths.size(); ++i) {
        // Test if this tour isn't processed already
        if (pDepot->IsConnectedTo(rGraph.vertices[paths[i]->second.front()].get()))
            continue;
        // Collect the vertex ids
        vids.push_back({ 0 });
        for (auto iterVid = paths[i]->second.begin(); iterVid != paths[i]->second.end(); iterVid++) {
            vids.back().push_back(*iterVid);
        }
        vids.back().push_back(0);
    }

    // Add the tour-edges to the graph
    if (vids.size() > rGraph.trucks)
        return -1;
    for (int i=0; i<vids.size(); ++i)
        rGraph.AddTourFromFile(vids[i]);

    return 0; // TODO: return tour value?
}


// "A guide to VR heurs" (page 5 of 12): in practice parallel savings heurs (meaning best feasable merge) works better than sequential
// Same for "classical and modern heurs for the VRP" (page 3 of 16)
