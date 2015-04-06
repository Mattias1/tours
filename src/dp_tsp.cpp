#include "dp_tsp.h"

#include "graph.h"
#include "treedecomposition.h"
#include "utils.h"
#include <algorithm>
#include <iostream>
#include <limits>
using namespace std;

//
//  All sorts of functions used to calculate the tour for the TSP problem.
//
vector<Edge*> tspDP(const TreeDecomposition& TD) {
    // Compute the smallest tour using DP on a tree decomposition.
    const Bag* pXroot = TD.getRoot();
    if (TD.vertices.size() < 1 || pXroot == nullptr || TD.getOriginalGraph()->vertices.size() < 1)
        return vector<Edge*>();

    // Prepare the tables (a hashlist for every bag)
    vector<unique_ptr<unordered_map<string, int>>> hashlists = vector<unique_ptr<unordered_map<string, int>>>(TD.vertices.size());
    for (unsigned int i=0; i<hashlists.size(); ++i)
        hashlists[i] = unique_ptr<unordered_map<string, int>>(new unordered_map<string, int>());

    // Start the calculation
    vector<int> degrees = vector<int>(pXroot->vertices.size(), 2);
    string S = fromDegreesEndpoints(degrees, vector<int>());
    int value = tspTable(*TD.getOriginalGraph(), hashlists, S, *pXroot);
    cout << "TSP cost: " << value << endl;
    for (unsigned int i=0; i<hashlists.size(); ++i) {
        cout << "X" << i << endl;
        for (const auto& iter : *hashlists[i])
            cout << "  " << iter.first << ": " << iter.second << endl;
    }

    // Reconstruct the tour
    if (value < numeric_limits<int>::max()) {
        const vector<Edge*>& tour = tspReconstruct(*TD.getOriginalGraph(), hashlists, S, *pXroot); // Used list(set( ... ))
        cout << "\nDP-TSP:\n  Length: " << value << "\n  Tour: ";
        for (unsigned int i=0; i<tour.size(); ++i)
            cout << tour[i] << ", ";
        cout << endl;
        return tour;
    }
    return vector<Edge*>();
}

int tspTable(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const string& S, const Bag& Xi) {
    // The smallest value such that all vertices below Xi have degree 2 and vertices in Xi have degrees defined by S
    // bool debug = false;
    // if debug: print("A({} {}, X{}): {}".format(toDegrees(S), toEndpoints(S), Xi.vid, "?"))

    // If we know the value already, just look it up.
    const auto& iter = rHashlists[Xi.vid]->find(S);
    if (iter != rHashlists[Xi.vid]->end())
        // if debug: print('lookup return: {}'.format(Xi.a[S]))
        return iter->second;

    // We don't know this value yet, so we compute it.
    vector<Edge*> edges;
    for (unsigned int i=0; i<Xi.vertices.size(); ++i) {
        Vertex* pV = Xi.vertices[i];
        for (unsigned int j=0; j<pV->edges.size(); ++j) {
            Edge* pE = pV->edges[j].get();
            if (!Xi.ContainsEdge(pE))
                continue;
            if (pV->vid < pE->Other(*pV)->vid)
                edges.push_back(pE);
        }
    }
    auto sortLambda = [&](Edge* pEdgeA, Edge* pEdgeB) {
        // Compare the costs of the edges
        return pEdgeA->Cost < pEdgeB->Cost;
    };
    sort(edges.begin(), edges.end(), sortLambda);
    vector<int> degrees = toDegrees(S);
    vector<int> endpoints = toEndpoints(S);
    vector<vector<int>> childEndpoints = vector<vector<int>>(Xi.edges.size());
    vector<vector<int>> childDegrees = vector<vector<int>>(Xi.edges.size()); //                   TODO TODO TODO: is vector<vector<...>> a good idea? because the vector inside may grow in size...
    for (unsigned int i=0; i<Xi.edges.size(); ++i) {
        childEndpoints[i] = vector<int>();
        childDegrees[i] = vector<int>(degrees.size(), 0);
    }
    (*rHashlists[Xi.vid])[S] = tspRecurse(graph, rHashlists, Xi, edges, 0, 0, degrees, childDegrees, endpoints, childEndpoints);
    // if debug: print('calculation return: {}'.format(Xi.a[S]))
    return rHashlists[Xi.vid]->at(S);
}

vector<Edge*> tspReconstruct(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const string& S, const Bag& Xi) {
    // Reconstruct the tsp tour (get a list of all edges)
    vector<Edge*> edges;
    for (unsigned int i=0; i<Xi.vertices.size(); ++i) {
        Vertex* pV = Xi.vertices[i];
        for (unsigned int j=0; j<pV->edges.size(); ++j) {
            Edge* pE = pV->edges[j].get();
            if (!Xi.ContainsEdge(pE))
                continue;
            if (pV->vid < pE->Other(*pV)->vid)
                edges.push_back(pE);
        }
    }
    auto sortLambda = [&](Edge* pEdgeA, Edge* pEdgeB) {
        // Compare the costs of the edges
        return pEdgeA->Cost < pEdgeB->Cost;
    };
    sort(edges.begin(), edges.end(), sortLambda);
    vector<int> degrees = toDegrees(S);
    vector<int> endpoints = toEndpoints(S);
    vector<vector<int>> childEndpoints = vector<vector<int>>(Xi.edges.size());
    vector<vector<int>> childDegrees = vector<vector<int>>(Xi.edges.size()); //                                             TODO TODO TODO: is vector<vector<...>> a good idea? because the vector inside may grow in size...
    for (unsigned int i=0; i<Xi.edges.size(); ++i) {
        childEndpoints[i] = vector<int>();
        childDegrees[i] = vector<int>(degrees.size(), 0);
    }
    return tspRecurseVector(graph, rHashlists, Xi, edges, 0, 0, degrees, childDegrees, endpoints, childEndpoints);
}

int tspChildEvaluation(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge*>& edges, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<int>& rEndpoints, vector<vector<int>>& rChildEndpoints, vector<Edge*>* pResultingEdgeList) {
    // This method is the base case for the calculate tsp recurse method.
    // If we analyzed the degrees of all vertices (i.e. we have a complete combination), return the sum of B values of all children.
    // bool debug = true;

    // Check: all bags (except the root) are not allowed to be a cycle.
    if (rEndpoints.size() == 0 and Xi.getParent() != nullptr)
        // if debug: print('{}All bags should be a cycle - no endpoints given'.format('  ' * len(Xi.vertices)))
        return numeric_limits<int>::max();

    // Base cost: the edges needed inside this Xi to account for the (target) degrees we didn't pass on to our children.
    vector<int> allChildEndpoints = flatten(rChildEndpoints);
    int val = tspEdgeSelect(numeric_limits<int>::max(), 0, graph, Xi, edges, rTargetDegrees, rEndpoints, allChildEndpoints, pResultingEdgeList);
    if (0 <= val && val < numeric_limits<int>::max()) {
        // if debug: print('{}Local edge selection cost: {}, edges: {}, degrees: {}, endpoints: {}, edgeList: {}'.format(
        //                               '  ' * len(Xi.vertices), val, edges, rTargetDegrees, endpoints, resultingEdgeList))
        for (unsigned int k=0; k<rChildDegrees.size(); ++k) {
            const vector<int>& cds = rChildDegrees[k];
            const Bag& Xkid = *dynamic_cast<Bag*>(Xi.edges[k]->Other(Xi));
            if (Xi.getParent() != &Xkid) {
                // Strip off the vertices not in Xkid and add degrees 2 for vertices not in Xi
                vector<int> kidDegrees(Xkid.vertices.size(), 2);
                for (unsigned int p=0; p<Xkid.vertices.size(); ++p)
                    for (unsigned int q=0; q<Xi.vertices.size(); ++q)
                        if (Xkid.vertices[p] == Xi.vertices[q])
                            kidDegrees[p] = cds[q];
                string S = fromDegreesEndpoints(kidDegrees, rChildEndpoints[k]);
                // if debug: print('{}child A: {}, cds: {}, degrees: {}, endpoints'.format('  ' * len(Xi.vertices),
                //                                               val, cds, kidDegrees, childEndpoints[k]))
                // Add to that base cost the cost of hamiltonian paths nescessary to satisfy the degrees.
                val += tspTable(graph, rHashlists, S, Xkid);
            }
        }
        // if debug: print('{}Min cost for X{} with these child-degrees: {}'.format('  ' * len(Xi.vertices), Xi.vid, val))
    }
    else {
        // if debug: print('{}No local edge selection found'.format('  ' * len(Xi.vertices)))
    }
    return val;
}

// TODO: the thing that uses tspLookback should be able to deal with a nullptr.
// TODO: worse, what if it's not a nullptr - who has ownership?
vector<Edge*> tspLookback(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge*>& edges, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<int>& rEndpoints, vector<vector<int>>& rChildEndpoints) {
    // This method is the base case for the reconstruct tsp recurse method.
    // bool debug = false;
    vector<Edge*> resultingEdgeList; // This list will be filled with the edges used in Xi
    vector<int> totalDegrees = duplicate(rTargetDegrees);
    for (unsigned int j=0; j<rChildDegrees.size(); ++j) {
        const vector<int>& cds = rChildDegrees[j];
        for (unsigned int i=0; i<cds.size(); ++i)
            totalDegrees[i] += cds[i];
    }

    string S = fromDegreesEndpoints(totalDegrees, rEndpoints);
    const auto& iter = rHashlists[Xi.vid]->find(S);
    if (iter == rHashlists[Xi.vid]->end())
        return vector<Edge*>();
    int val = iter->second;
    if (val != tspChildEvaluation(graph, rHashlists, Xi, edges, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints, &resultingEdgeList))
        return vector<Edge*>(); // Side effect above intended to fill the edge list
    // if debug: print('X{} edgelist 1: {}'.format(Xi.vid, resultingEdgeList))
    // So these are indeed the child degrees that we are looking for
    for (unsigned int k=0; k<rChildDegrees.size(); ++k) {
        const vector<int>& cds = rChildDegrees[k];

        const Bag& Xkid = *dynamic_cast<Bag*>(Xi.edges[k]->Other(Xi));
        if (Xi.getParent() != &Xkid) {
            // Strip off the vertices not in Xkid and add degrees 2 for vertices not in Xi
            vector<int> kidDegrees(Xkid.vertices.size(), 2);
            for (unsigned int p=0; p<Xkid.vertices.size(); ++p)
                for (unsigned int q=0; q<Xi.vertices.size(); ++q)
                    if (Xkid.vertices[p] == Xi.vertices[q])
                        kidDegrees[p] = cds[q];
            string S = fromDegreesEndpoints(kidDegrees, rChildEndpoints[k]);
            // We already got the resultingEdgeList for Xi, now add the REL for all the children
            const vector<Edge*>& test = tspReconstruct(graph, rHashlists, S, Xkid);
            pushBackList(&resultingEdgeList, test);
            // print('test 2 edgelist: {}'.format(resultingEdgeList))
        }
    }
    // if debug: print('X{} edgelist 3: {}'.format(Xi.vid, resultingEdgeList))
    return resultingEdgeList;
}

int tspRecurse(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge*>& edges, unsigned int i, unsigned int j, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<int>& rEndpoints, vector<vector<int>>& rChildEndpoints) {
    // Select all possible mixes of degrees for all vertices and evaluate them
    //   i = the vertex we currently analyze, j = the child we currently analyze
    //   rTargetDegrees goes from full to empty, rChildDegrees from empty to full, endpoints are the endpoints for each child path
    bool debug = true;
    if (debug) {
        cout << dbg("  ", i) << dbg(rChildDegrees) << dbg("  ", Xi.vertices.size() + 10 - i);
        cout << "(X" << Xi.vid << ": " << i << ", " << j << ")     " << dbg(rTargetDegrees) << "|" << dbg(rEndpoints) << endl;
    }

    // Final base case.
    if (i >= Xi.vertices.size())
        return tspChildEvaluation(graph, rHashlists, Xi, edges, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints); // BaseF
    // Base case: if we can't or didn't want to 'spend' this degree, move on
    if (rTargetDegrees[i] == 0 || j >= Xi.edges.size())
        return tspRecurse(graph, rHashlists, Xi, edges, i + 1, 0, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints);
    const Bag& Xj = *dynamic_cast<Bag*>(Xi.edges[j]->Other(Xi));
    // Base case: if the current bag (must be child) does not contain the vertex to analyze, try the next (child) bag
    if (Xi.getParent() == Xi.edges[j]->Other(Xi) || !Xj.ContainsVertex(Xi.vertices[i]))
        return tspRecurse(graph, rHashlists, Xi, edges, i, j + 1, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints);

    // If the current degree is 2, try letting the child manage it
    int result = numeric_limits<int>::max();
    if (rTargetDegrees[i] == 2 and rChildDegrees[j][i] == 0) {
        vector<int> td = duplicate(rTargetDegrees);
        vector<vector<int>> cds = duplicate(rChildDegrees);
        td[i] = 0;
        cds[j][i] = 2;
        result = tspRecurse(graph, rHashlists, Xi, edges, i + 1, 0, td, cds, rEndpoints, rChildEndpoints);
    }
    // If the current degree is at least 1 (which it is if we get here),
    //   try to combine it (for all other vertices) in a hamiltonian path
    for (unsigned int k=i+1; k<Xi.vertices.size(); ++k) {
        // Stay in {0, 1, 2}
        if (rTargetDegrees[k] < 1 || rChildDegrees[j][k] > 1 || !Xj.ContainsVertex(Xi.vertices[k]))
            continue;
        // Don't add edges twice
        if (inEndpoints(rChildEndpoints[j], Xi.vertices[i]->vid, Xi.vertices[k]->vid))
            continue;
        vector<int> td = duplicate(rTargetDegrees);
        vector<vector<int>> cds = duplicate(rChildDegrees);
        vector<vector<int>> eps = duplicate(rChildEndpoints);
        td[i] -= 1;
        cds[j][i] += 1;
        td[k] -= 1;
        cds[j][k] += 1;
        eps[j].push_back(Xi.vertices[i]->vid);
        eps[j].push_back(Xi.vertices[k]->vid);
        // We may have to try to analyze the same vertex again if it's degree is higher than 1
        result = min(result, tspRecurse(graph, rHashlists, Xi, edges, i, j, td, cds, rEndpoints, eps));
    }
    // Also, try not assigning this degree to anyone, we (maybe) can solve it inside Xi
    result = min(result, tspRecurse(graph, rHashlists, Xi, edges, i, j + 1, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints));
    return result;
}

vector<Edge*> tspRecurseVector(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge*>& edges, unsigned int i, unsigned int j, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<int>& rEndpoints, vector<vector<int>>& rChildEndpoints) {
    // Select all possible mixes of degrees for all vertices and evaluate them
    //   i = the vertex we currently analyze, j = the child we currently analyze
    //   rTargetDegrees goes from full to empty, rChildDegrees from empty to full, endpoints are the endpoints for each child path
    // bool debug = false;
    // if debug: print('{}{}{}     (X{}: {}, {})   {}|{}'.format('  ' * i, rChildDegrees, '  ' * (len(Xi.vertices) + 8 - i), Xi.vid, i, j, rTargetDegrees, rEndpoints))
    // Final base case.
    if (i >= Xi.vertices.size())
        return tspLookback(graph, rHashlists, Xi, edges, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints); // BaseF
    // Base case: if we can't or didn't want to 'spend' this degree, move on
    if (rTargetDegrees[i] == 0 || j >= Xi.edges.size())
        return tspRecurseVector(graph, rHashlists, Xi, edges, i + 1, 0, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints);
    const Bag& Xj = *dynamic_cast<Bag*>(Xi.edges[j]->Other(Xi));
    // Base case: if the current bag (must be child) does not contain the vertex to analyze, try the next (child) bag
    if (Xi.getParent() == Xi.edges[j]->Other(Xi) || !Xj.ContainsVertex(Xi.vertices[i]))
        return tspRecurseVector(graph, rHashlists, Xi, edges, i, j + 1, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints);

    // If the current degree is 2, try letting the child manage it
    vector<Edge*> result;
    if (rTargetDegrees[i] == 2 and rChildDegrees[j][i] == 0) {
        vector<int> td = duplicate(rTargetDegrees);
        vector<vector<int>> cds = duplicate(rChildDegrees);
        td[i] = 0;
        cds[j][i] = 2;
        result = tspRecurseVector(graph, rHashlists, Xi, edges, i + 1, 0, td, cds, rEndpoints, rChildEndpoints);
    }
    // If the current degree is at least 1 (which it is if we get here),
    //   try to combine it (for all other vertices) in a hamiltonian path
    for (unsigned int k=i+1; k<Xi.vertices.size(); ++k) {
        // Stay in {0, 1, 2}
        if (rTargetDegrees[k] < 1 || rChildDegrees[j][k] > 1 || !Xj.ContainsVertex(Xi.vertices[k]))
            continue;
        // Don't add edges twice
        if (inEndpoints(rChildEndpoints[j], Xi.vertices[i]->vid, Xi.vertices[k]->vid))
            continue;
        vector<int> td = duplicate(rTargetDegrees);
        vector<vector<int>> cds = duplicate(rChildDegrees);
        vector<vector<int>> eps = duplicate(rChildEndpoints);
        td[i] -= 1;
        cds[j][i] += 1;
        td[k] -= 1;
        cds[j][k] += 1;
        eps[j].push_back(Xi.vertices[i]->vid);
        eps[j].push_back(Xi.vertices[k]->vid);
        // We may have to try to analyze the same vertex again if it's degree is higher than 1
        pushBackList(&result, tspRecurseVector(graph, rHashlists, Xi, edges, i, j, td, cds, rEndpoints, eps));
    }
    // Also, try not assigning this degree to anyone, we (maybe) can solve it inside Xi
    pushBackList(&result, tspRecurseVector(graph, rHashlists, Xi, edges, i, j + 1, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints));
    return result;
}

// Todo: use the minimum to abort early??? (is possible for leaf case, but perhaps not for normal bag case
int tspEdgeSelect(int minimum, unsigned int index, const Graph& graph, const Bag& Xi, const vector<Edge*>& edges, const vector<int>& degrees, vector<int>& rEndpoints, vector<int>& rAllChildEndpoints, vector<Edge*>* pEdgeList) {
    // bool debug = false;
    // Calculate the smallest cost to satisfy the degrees target using only using edges >= the index
    // Base case 1: the degrees are all zero, so we succeeded as we don't need to add any more edges
    bool satisfied = true;
    for (unsigned int i=0; i<degrees.size(); ++i) {
        if (degrees[i] != 0) {
            satisfied = false;
            break;
        }
    }
    if (satisfied) {
        // So we have chosen all our edges and satisfied the targets - now make sure there is no cycle (unless root)
        if (!cycleCheck(graph, rEndpoints, pEdgeList, rAllChildEndpoints))
            // if debug: print('Edge select ({}): edges contain a cycle'.format(index))
            return numeric_limits<int>::max();
        // if debug: print('Edge select ({}): no need to add edges, min value: 0'.format(index))
        return 0;
    }
    // Base case 2: we have not succeeded yet, but there are no more edges to add, so we failed
    if (index >= edges.size())
        // if debug: print('Edge select ({}): no more edges to add'.format(index))
        return numeric_limits<int>::max();
    // Base case 3: one of the degrees is < 1, so we added too many vertices, so we failed [with side effect]
    Edge* pEdge = edges[index];
    vector<int> deg = duplicate(degrees);
    int assertCounter = 0;
    for (unsigned int i=0; i<deg.size(); ++i) {
        int d = deg[i];
        if (Xi.vertices[i] == pEdge->pA || Xi.vertices[i] == pEdge->pB) {
            if (d < 0) // If it's negative it will tell us later
                       //  - can't return right now, as we need to evaluete not taking this edge as well.
                // if debug: print('Edge select ({}): too many edges added'.format(index))
                return numeric_limits<int>::max();
            // While checking this base case, also compute the new degree list for the first recursion
            deg[i] -= 1;
            assertCounter += 1;
        }
    }
    if (assertCounter != 0 && assertCounter != 2)
        cout << "ASSERTION ERROR - the assertCounter is not 0 or 2." << endl;
    // Try both to take the edge and not to take the edge
    // if debug: print('Edge select ({}), degrees: {}'.format(index, degrees))
    vector<Edge*> tempEL1;
    vector<Edge*> tempEL2;
    vector<Edge*>* pTempEL1 = nullptr;
    vector<Edge*>* pTempEL2 = nullptr;
    if (pEdgeList != nullptr) {
        tempEL1 = duplicate(*pEdgeList);        // Notice the pointers to stack objects.
        tempEL2 = duplicate(tempEL1);           // They will go out of scope at the end of the function, but that's fine,
        pTempEL1 = &tempEL1;                    // we no longer need them after the edges are pushed back to pEdgeList.
        pTempEL2 = &tempEL2;                    //
        pTempEL1->push_back(pEdge);
    }
    int val = tspEdgeSelect(minimum - pEdge->Cost, index + 1, graph, Xi, edges, deg, rEndpoints, rAllChildEndpoints, pTempEL1);
    if (val < numeric_limits<int>::max())
        minimum = min(minimum, pEdge->Cost + val);
    val = tspEdgeSelect(minimum, index + 1, graph, Xi, edges, degrees, rEndpoints, rAllChildEndpoints, pTempEL2);
    if (val < minimum) {
        minimum = val;
        // So without edge is better - Append the second edge list
        if (pEdgeList != nullptr)
            pushBackList(pEdgeList, *pTempEL2);
    }
    // So without edge is not better - Append the first edge list
    else if (pEdgeList != nullptr) {
        pushBackList(pEdgeList, *pTempEL1);
    }
    // if debug: print('Edge select ({}): min value: {}, edges: {}'.format(index, minimum, edgeList))
    return minimum;
}

vector<int> toDegrees(const string& S) {
    // From a string representation to a list of degrees
    return splitInt(split(S, '|')[0], ',');
}

vector<int> toEndpoints(const string& S) {
    // From a string representation to a list of edges
    vector<string> tempSplit = split(S, '|');
    if (tempSplit.size() < 2)
        return vector<int>();
    return splitInt(tempSplit[1], ',');
}

string fromDegreesEndpoints(const vector<int>& degrees, const vector<int>& endpoints) {
    // From a list of degrees and endpoints to a string representation
    return join(degrees, ',') + '|' + join(endpoints, ',');
}

bool cycleCheck(const Graph& graph, vector<int>& rEndpoints, vector<Edge*>* pEdgeList, vector<int>& rAllChildEndpoints) {
    // This method returns whether or not the given edge list and all child endpoints provide a set of paths
    // satisfying the endpoints and sorts the edge list in place.
    int progressCounter = -2;
    unsigned int edgeCounter = 0;
    unsigned int endpsCounter = 0;
    Vertex* pV = nullptr;

    vector<Edge*> edgeList;
    if (pEdgeList == nullptr)
        edgeList = vector<Edge*>(); // Note that pEdgeList still is null, so I should use the variable edgeList everywhere.
    else
        edgeList = *pEdgeList; // Ok, this bit of code feels weird, because now I have a 'local' variable on the heap. It should work though. I think.
    // bool debug = false;

    // Special case: the root bag.
    if (rEndpoints.size() == 0) {
        if (rAllChildEndpoints.size() > 0) {
            rEndpoints.push_back(rAllChildEndpoints[0]);
            rEndpoints.push_back(rAllChildEndpoints[1]);
            endpsCounter += 2;
        }
        else if (edgeList.size() > 0) {
            rEndpoints.push_back(edgeList[0]->pA->vid);
            rEndpoints.push_back(edgeList[0]->pB->vid);
            edgeCounter += 1;
        }
        else {
            // if debug: print('ERROR: cycle check root bag has both no edges to add, nor any child endpoints')
            return false;
        }
    }

    // Normal case
    while (true) {
        // Dump the state
        // if debug:
            // print('cycle check dump 1:')
            // print('  endpoints: {}'.format(endpoints))
            // print('  edgeList: {} - {}'.format(edgeCounter, edgeList))
            // print('  kid endpoints: {} - {}'.format(endpsCounter, rAllChildEndpoints))
            // print('  progress: {} - v: {}\n'.format(progressCounter, -1 if not v else v.vid))

        // If we completed the path
        if (pV == nullptr || pV->vid == rEndpoints[progressCounter + 1]) {
            progressCounter += 2;
            if (static_cast<unsigned int>(progressCounter) >= rEndpoints.size()) {
                if (edgeCounter == edgeList.size() && endpsCounter == rAllChildEndpoints.size())
                    return true;
                else
                    // if debug: print('ERROR: all endpoints are satisfied, but there are edges or endpoints left')
                    return false;
            }
            pV = graph.vertices[rEndpoints[progressCounter]].get();
        }

        // Dump the state
        // if debug:
            // print('cycle check dump 2:')
            // print('  endpoints: {}'.format(endpoints))
            // print('  edgeList: {} - {}'.format(edgeCounter, edgeList))
            // print('  kid endpoints: {} - {}'.format(endpsCounter, rAllChildEndpoints))
            // print('  progress: {} - v: {}\n'.format(progressCounter, -1 if not v else v.vid))

        // Find the next vertex
        bool noBreak = true;
        for (unsigned int i=endpsCounter; i<rAllChildEndpoints.size(); i += 2) {
            if (rAllChildEndpoints[i] == pV->vid || rAllChildEndpoints[i + 1] == pV->vid) {
                pV = graph.vertices[rAllChildEndpoints[pV->vid == rAllChildEndpoints[i] ? i + 1 : i]].get();

                int temp = rAllChildEndpoints[endpsCounter];
                rAllChildEndpoints[endpsCounter] = rAllChildEndpoints[i];
                rAllChildEndpoints[i] = temp;
                temp = rAllChildEndpoints[endpsCounter + 1];
                rAllChildEndpoints[endpsCounter + 1] = rAllChildEndpoints[i + 1];
                rAllChildEndpoints[i + 1] = temp;

                endpsCounter += 2;
                noBreak = false;
                break;
            }
        }
        if (noBreak) {
            noBreak = true;
            for (unsigned int i=0; i<edgeList.size(); ++i) {
                if (edgeList[i]->IsIncidentTo(pV)) {
                    pV = edgeList[i]->Other(*pV);
                    Edge* pTemp = edgeList[edgeCounter];
                    edgeList[edgeCounter] = edgeList[i];
                    edgeList[i] = pTemp;
                    edgeCounter++;
                    noBreak = false;
                    break;
                }
            }
            if (noBreak) {
                // if debug: print('eps: {}, edgelist: {}, all kid eps: {}'.format(endpoints, edgeList, rAllChildEndpoints))
                // if debug: print('ERROR, no more endpoints or edges found according to specs')
                return false;
            }
        }
    }
    cout << "ERROR: The code should not come here (cycleCheck function)." << endl;
    return false;
}

bool inEndpoints(const vector<int>& endpoints, int start, int end) {
    // Return whether or not this combination of endpoints (or reversed order) is already in the endpoints list
    for (unsigned int j=0; j<endpoints.size(); j += 2)
        if ((endpoints[j] == start and endpoints[j + 1] == end) || (endpoints[j + 1] == start and endpoints[j] == end))
            return true;
    return false;
}

