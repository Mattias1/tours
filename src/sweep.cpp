#include "sweep.h"

#include <vector>
#include <cmath>
#include <algorithm>

#include "use_lkh.h"
#include "utils.h"

inline float vertexAngle(const Graph& graph, int vid) {
    return atan2(graph.vertices[vid]->y - graph.vertices[0]->y, graph.vertices[vid]->x - graph.vertices[0]->x);
}

int sweep(Graph& rGraph, int startVid /*=-1*/) {
    // If confronted with a undefined start vertex id (the one that has the 0-angle), choose a random one (haha)
    if (startVid <= 0)
        startVid = 1;

    // Compute and sort all the angles (except for the depot obviously)
    float baseAngle = vertexAngle(rGraph, startVid);
    vector<pair<int, float>> angles;
    for (int i=1; i<rGraph.vertices.size(); ++i) {
        float angle = vertexAngle(rGraph, i);
        if (angle < baseAngle)
            angle += 2 * M_PI;
        angles.push_back(make_pair(i, angle));
    }
    auto sortLambda = [&](const pair<int, float>& a, const pair<int, float>& b) {
        return a.second > b.second;
    };
    sort(angles.begin(), angles.end(), sortLambda);

    // Group all vertices (determine which vertex belongs to which tour)
    vector<vector<int>> vids = {{ 0 }};
    int totalDemand = 0;
    for (int i=1; i<rGraph.vertices.size(); ++i) {
        if (totalDemand + rGraph.vertices[i]->demand > rGraph.capacity) {
            // Start a new tour (for a new truck)
            vids.push_back({ 0 });
            totalDemand = 0;
        }
        vids.back().push_back(i);
        totalDemand += rGraph.vertices[i]->demand;
    }

    // Create a good solution for each of the vertex groups (in other words: solve the TSP for each of the groups)
    if (vids.size() > rGraph.trucks)
        return -1;
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
