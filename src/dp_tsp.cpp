#include "dp_tsp.h"

#include "graph.h"
#include "treedecomposition.h"
#include "utils.h"
#include <algorithm>
#include <iostream>
#include <limits>
using namespace std;

//
//  The core functions used to calculate the tour for the TSP problem.
//
vector<Edge*> tspDP(const TreeDecomposition& TD, bool consoleOutput /*=true*/) {
    // Compute the smallest tour using DP on a tree decomposition.
    bool debug = false;

    // Make sure there is a proper graph and decomposition
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
    if (consoleOutput)
        cout << "TSP cost: " << value << endl;
    if (debug) {
        for (unsigned int i=0; i<hashlists.size(); ++i) {
            cout << "X" << i << endl;
            for (const auto& iter : *hashlists[i])
                cout << "  " << iter.first << ": " << iter.second << endl;
        }
    }

    // Reconstruct the tour
    if (value < numeric_limits<int>::max()) {
        vector<Edge*> tour = removeDoubles(tspReconstruct(*TD.getOriginalGraph(), hashlists, S, *pXroot), TD.getOriginalGraph()->vertices.size());
        // vector<int> allChildEndpointsParameter;
        // cycleCheck(*TD.getOriginalGraph(), vector<int>(), &tour, allChildEndpointsParameter); // Sort the tour - TODO - ERROR, IT DOESN'T ACTUALLY SORT IT
        if (debug) {
            cout << "\nDP-TSP:\n  Length: " << value << "\n  Tour: ";
            for (unsigned int i=0; i<tour.size(); ++i)
                cout << *tour[i] << ", ";
            cout << endl;
        }
        return tour;
    }
    return vector<Edge*>();
}

int tspTable(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const string& S, const Bag& Xi) {
    // The smallest value such that all vertices below Xi have degree 2 and vertices in Xi have degrees defined by S

    // If we know the value already, just look it up
    const auto& iter = rHashlists[Xi.vid]->find(S);
    if (iter != rHashlists[Xi.vid]->end())
        return iter->second;

    // We don't know this value yet, so we compute it (first get all parameters correct)
    vector<Edge*> edges = getBagEdges(Xi);
    vector<int> degrees = toDegrees(S);
    vector<int> endpoints = toEndpoints(S);
    vector<vector<int>> childEndpoints = vector<vector<int>>(Xi.edges.size());
    vector<vector<int>> childDegrees = vector<vector<int>>(Xi.edges.size());
    for (unsigned int i=0; i<Xi.edges.size(); ++i) {
        childEndpoints[i] = vector<int>();
        childDegrees[i] = vector<int>(degrees.size(), 0);
    }
    // Recursively find all possible combinations that satisfy the parameter arrays
    (*rHashlists[Xi.vid])[S] = tspRecurse(graph, rHashlists, Xi, edges, 0, 0, degrees, childDegrees, endpoints, childEndpoints);
    return rHashlists[Xi.vid]->at(S);
}

vector<Edge*> tspReconstruct(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const string& S, const Bag& Xi) {
    // Reconstruct the tsp tour (get a list of all tour edges)
    vector<Edge*> edges = getBagEdges(Xi);
    vector<int> degrees = toDegrees(S);
    vector<int> endpoints = toEndpoints(S);
    vector<vector<int>> childEndpoints = vector<vector<int>>(Xi.edges.size());
    vector<vector<int>> childDegrees = vector<vector<int>>(Xi.edges.size());
    for (unsigned int i=0; i<Xi.edges.size(); ++i) {
        childEndpoints[i] = vector<int>();
        childDegrees[i] = vector<int>(degrees.size(), 0);
    }
    return tspRecurseVector(graph, rHashlists, Xi, edges, 0, 0, degrees, childDegrees, endpoints, childEndpoints);
}

int tspChildEvaluation(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge*>& edges, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<int>& rEndpoints, const vector<vector<int>>& rChildEndpoints, vector<Edge*>* pResultingEdgeList /*=nullptr*/) {
    // This method is the base case for the calculate tsp recurse method.
    // If we analyzed the degrees of all vertices (i.e. we have a complete combination), return the sum of B values of all children.
    bool debug = false;

    // Check: all bags (except the root) are not allowed to be a cycle.
    if (rEndpoints.size() == 0 and Xi.getParent() != nullptr) {
        if (debug) {
            cout << dbg("  ", Xi.vertices.size()) << "All bags (except root) should have at least one path - no endpoints given" << endl;
        }
        return numeric_limits<int>::max();
    }

    // Base cost: the edges needed inside this Xi to account for the (target) degrees we didn't pass on to our children.
    vector<int> allChildEndpoints = flatten(rChildEndpoints);
    unsigned int edgeListBits = 0;
    int val = tspEdgeSelect(numeric_limits<int>::max(), 0, graph, Xi, edges, rTargetDegrees, rEndpoints, allChildEndpoints, &edgeListBits);
    if (pResultingEdgeList != nullptr)
        addToEdgeListFromBits(edges, pResultingEdgeList, edgeListBits);
    if (0 <= val && val < numeric_limits<int>::max()) { // TODO: why can val ever be < 0??? Why is this first part of the check here??? It is also there in the python version...
        if (debug) {
            cout << dbg("  ", Xi.vertices.size()) << "Local edge selection cost: " << val << ", edges: " << dbg(edges) << ", degrees: " << dbg(rTargetDegrees);
            cout << ", endpoints: " << dbg(rEndpoints) << ", edgeList: " << dbg(pResultingEdgeList) << endl;
        }
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
                if (debug) {
                    cout << dbg("  ", Xi.vertices.size()) << "child A: " << val << ", cds: " << dbg(cds) << ", degrees: " << dbg(kidDegrees);
                    cout << ", endpoints: " << dbg(rChildEndpoints[k]) << endl;
                }
                // Add to that base cost the cost of hamiltonian paths nescessary to satisfy the degrees.
                int tableVal = tspTable(graph, rHashlists, S, Xkid);
                if (tableVal == numeric_limits<int>::max())
                    return tableVal;
                val += tableVal;
            }
        }
        if (debug) {
            cout << dbg("  ", Xi.vertices.size()) << "Min cost for X" << Xi.vid << " with these child-degrees: " << val << endl;
        }
    }
    else if (debug) {
        cout << dbg("  ", Xi.vertices.size()) << "No local edge selection found" << endl;
    }
    return val;
}

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
    bool debug = false;
    if (debug) {
        // tree-of-childDegrees          (Xi: i, j)   targetDegrees|endpoints
        cout << dbg("  ", i) << dbg(rChildDegrees) << dbg("  ", Xi.vertices.size() + 9 - i);
        cout << "(X" << Xi.vid << ": " << i << ", " << j << ")   " << dbg(rTargetDegrees) << "|" << dbg(rEndpoints) << endl;
    }

    // Final base case.
    if (i >= Xi.vertices.size())
        return tspChildEvaluation(graph, rHashlists, Xi, edges, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints);
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
        // Update the degrees
        td[i] -= 1;
        cds[j][i] += 1;
        td[k] -= 1;
        cds[j][k] += 1;
        // Update the endpoints (update when one (or both) of the endpoints are already in the list, otherwise insert)
        if (cds[j][i] == 2 || cds[j][k] == 2) {
            // So now at least one of the two is not new in the list, so we update it (or them)
            // Find out who's in the list (and where)
            int vidI = Xi.vertices[i]->vid;
            int vidK = Xi.vertices[k]->vid;
            unsigned int posI = numeric_limits<unsigned int>::max();
            unsigned int posK = numeric_limits<unsigned int>::max(); // The position of the vertex in the list that has the same vertex id as the k-th vertex we compare with
            for (unsigned int l=0; l<eps[j].size(); ++l) {
                if (eps[j][l] == vidI)
                    posI = l;
                else if (eps[j][l] == vidK)
                    posK = l;
            }
            // Two cases (1): only one is already in there (in this case the path is extended) (branch on which one is new)
            if (posI == numeric_limits<unsigned int>::max()) {
                if (posK == numeric_limits<unsigned int>::max())
                    cout << "ASSERTION ERROR - tspRecurse - posK == uint::max !!!" << endl;
                eps[j][posK] = vidI;
            }
            else if (posK == numeric_limits<unsigned int>::max()) {
                eps[j][posI] = vidK;
            }
            // or (2): either both are already in there (in that case this edge merges the two paths together)
            else {
                unsigned int posBeginK = posK % 2 == 0? posK : posK - 1;
                unsigned int posOtherK = posK == posBeginK? posBeginK + 1 : posBeginK;
                eps[j][posI] = eps[j][posOtherK];
                eps[j].erase(eps[j].begin() + posBeginK, eps[j].begin() + posBeginK + 2);
            }
        }
        else {
            // So now both are new in the list, so we just add (insert) them
            eps[j].push_back(Xi.vertices[i]->vid);
            eps[j].push_back(Xi.vertices[k]->vid);
        }

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

    // If the current degree is 2, try letting the child manage it (it = the i-th vertex in Xi)
    vector<Edge*> result;
    if (rTargetDegrees[i] == 2 and rChildDegrees[j][i] == 0) {
        vector<int> td = duplicate(rTargetDegrees);
        vector<vector<int>> cds = duplicate(rChildDegrees);
        td[i] = 0;
        cds[j][i] = 2;
        result = tspRecurseVector(graph, rHashlists, Xi, edges, i + 1, 0, td, cds, rEndpoints, rChildEndpoints);
    }
    // If the current degree is at least 1 (which it is if we get here), try to combine it (for all other vertices) in a hamiltonian path
    for (unsigned int k=i+1; k<Xi.vertices.size(); ++k) {
        // Stay in {0, 1, 2} (and make sure child k has the i-th vertex of Xi as well)
        if (rTargetDegrees[k] < 1 || rChildDegrees[j][k] > 1 || !Xj.ContainsVertex(Xi.vertices[k]))
            continue;
        // Don't add edges twice
        if (inEndpoints(rChildEndpoints[j], Xi.vertices[i]->vid, Xi.vertices[k]->vid))
            continue;
        vector<int> td = duplicate(rTargetDegrees);
        vector<vector<int>> cds = duplicate(rChildDegrees);
        vector<vector<int>> eps = duplicate(rChildEndpoints);
        // Update the degrees
        td[i] -= 1;
        cds[j][i] += 1;
        td[k] -= 1;
        cds[j][k] += 1;
        // Update the endpoints (update when one (or both) of the endpoints are already in the list, otherwise insert)
        if (cds[j][i] == 2 || cds[j][k] == 2) {
            // So now at least one of the two is not new in the list, so we update it (or them)
            // Find out who's in the list (and where)
            int vidI = Xi.vertices[i]->vid;
            int vidK = Xi.vertices[k]->vid;
            unsigned int posI = numeric_limits<unsigned int>::max();
            unsigned int posK = numeric_limits<unsigned int>::max(); // The position of the vertex in the list that has the same vertex id as the k-th vertex we compare with
            for (unsigned int l=0; l<eps[j].size(); ++l) {
                if (eps[j][l] == vidI)
                    posI = l;
                else if (eps[j][l] == vidK)
                    posK = l;
            }
            // Two cases (1): only one is already in there (in this case the path is extended) (branch on which one is new)
            if (posI == numeric_limits<unsigned int>::max()) {
                if (posK == numeric_limits<unsigned int>::max())
                    cout << "ASSERTION ERROR - tspRecurse - posK == uint::max !!!" << endl;
                eps[j][posK] = vidI;
            }
            else if (posK == numeric_limits<unsigned int>::max()) {
                eps[j][posI] = vidK;
            }
            // or (2): either both are already in there (in that case this edge merges the two paths together)
            else {
                unsigned int posBeginK = posK % 2 == 0? posK : posK - 1;
                unsigned int posOtherK = posK == posBeginK? posBeginK + 1 : posBeginK;
                eps[j][posI] = eps[j][posOtherK];
                eps[j].erase(eps[j].begin() + posBeginK, eps[j].begin() + posBeginK + 2);
            }
        }
        else {
            // So now both are new in the list, so we just add (insert) them
            eps[j].push_back(Xi.vertices[i]->vid);
            eps[j].push_back(Xi.vertices[k]->vid);
        }

        // We may have to try to analyze the same vertex again if it's degree is higher than 1
        pushBackList(&result, tspRecurseVector(graph, rHashlists, Xi, edges, i, j, td, cds, rEndpoints, eps));
    }
    // Also, try not assigning this degree to anyone, we (maybe) can solve it inside Xi
    pushBackList(&result, tspRecurseVector(graph, rHashlists, Xi, edges, i, j + 1, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints));
    return result;
}

// Todo: use the minimum to abort early??? (is possible for leaf case, but perhaps not for normal bag case
int tspEdgeSelect(int minimum, unsigned int index, const Graph& graph, const Bag& Xi, const vector<Edge*>& edges, const vector<int>& degrees, vector<int>& rEndpoints, vector<int>& rAllChildEndpoints, unsigned int* pEdgeListBits /*=nullptr*/) {
    // Calculate the smallest cost to satisfy the degrees target using only using edges >= the index
    bool debug = false;

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
        vector<Edge*> edgeList;
        addToEdgeListFromBits(edges, &edgeList, *pEdgeListBits);
        if (!cycleCheck(graph, rEndpoints, &edgeList, rAllChildEndpoints)) { // Side effect: edgeList and child endpoints can be 'sorted'
            if (debug)
                cout << "Edge select (" << index << "): edges contain a cycle" << endl;
            return numeric_limits<int>::max();
        }
        if (debug)
            cout << "Edge select (" << index << "): satisfied: no need to add any more edges, min value: 0" << endl;
        return 0;
    }

    // Base case 2: we have not succeeded yet, but there are no more edges to add, so we failed
    if (index >= edges.size()) {
        if (debug)
            cout << "Edge select (" << index << "): no more edges to add" << endl;
        return numeric_limits<int>::max();
    }

    // Base case 3: one of the degrees is < 1, so we added too many vertices, so we failed [with side effect]
    Edge* pEdge = edges[index];
    vector<int> deg = duplicate(degrees);
    int assertCounter = 0;
    for (unsigned int i=0; i<deg.size(); ++i) {
        if (Xi.vertices[i] == pEdge->pA || Xi.vertices[i] == pEdge->pB) {
            if (deg[i] < 0) {   // If it is negative already (we substract 1 after this if statement) then we can return int::max now.
                                // If it becomes negative later we will just continue, as we have to evaluate not taking this edge as well.
                                // (we return int::max for that case (taking the edge) in this piece of code, but in the next function call).
                if (debug)
                    cout << "Edge select (" << index << "): too many edges added" << endl;
                return numeric_limits<int>::max();
            }
            // While checking this base case, also compute the new degree list for the first recursion
            deg[i] -= 1;
            assertCounter += 1;
        }
    }
    if (assertCounter != 0 && assertCounter != 2)
        cout << "ASSERTION ERROR - the assertCounter is not 0 or 2." << endl;

    // Try both to take the edge and not to take the edge
    if (debug)
        cout << "Edge select (" << index << "), degrees: " << dbg(degrees) << endl;
    unsigned int tempEL1, tempEL2;
    if (pEdgeListBits != nullptr) {
        tempEL1 = *pEdgeListBits;
        tempEL2 = tempEL1;
    }
    tempEL1 |= 1 << index;
    int val = tspEdgeSelect(minimum - pEdge->Cost, index + 1, graph, Xi, edges, deg, rEndpoints, rAllChildEndpoints, &tempEL1);
    if (val < numeric_limits<int>::max())
        minimum = min(minimum, pEdge->Cost + val);
    val = tspEdgeSelect(minimum, index + 1, graph, Xi, edges, degrees, rEndpoints, rAllChildEndpoints, &tempEL2);
    if (val < minimum) {
        minimum = val;
        // So without edge is better - Append the second edge list
        if (pEdgeListBits != nullptr)
            *pEdgeListBits |= tempEL2;
    }
    // So without edge is not better - Append the first edge list
    else if (pEdgeListBits != nullptr) {
        *pEdgeListBits |= tempEL1;
    }
    if (debug)
        cout << "Edge select (" << index << "): min value: " << minimum << ", edges: " << *pEdgeListBits << endl;
    return minimum;
}

//
// Some helper functions - also used in the VRP DP
//
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

bool cycleCheck(const Graph& graph, const vector<int>& endpoints, vector<Edge*>* pEdgeList, vector<int>& rAllChildEndpoints, int stepsize /*=2*/) {
    // This method returns whether or not the given edge list and all child endpoints provide a set of paths
    // satisfying the endpoints and sorts the edge list in place.
    bool debug = false;

    // Inits
    int progressCounter = -stepsize;
    unsigned int edgeCounter = 0;
    unsigned int endpsCounter = 0;
    Vertex* pV = nullptr;
    vector<int> endpointsClone = duplicate(endpoints);

    vector<Edge*> edgeList;
    if (pEdgeList == nullptr)
        edgeList = vector<Edge*>(); // Note that pEdgeList still is null, so I should use the variable edgeList everywhere in this method.
    else
        edgeList = *pEdgeList; // If I understand C++ well, this invokes the copy constructor. That'd be good.

    // Special init case for the root bag.
    if (endpointsClone.size() == 0) {
        if (rAllChildEndpoints.size() > 0) {
            endpointsClone.push_back(rAllChildEndpoints[0]);
            endpointsClone.push_back(rAllChildEndpoints[1]);
            endpsCounter += 2;
        }
        else if (edgeList.size() > 0) {
            endpointsClone.push_back(edgeList[0]->pA->vid);
            endpointsClone.push_back(edgeList[0]->pB->vid);
            ++edgeCounter;
        }
        else {
            if (debug)
                cout << "ERROR: cycle check root bag has both no edges to add, nor any child endpoints" << endl;
            return false;
        }
    }

    // Normal case
    while (true) {
        // Dump the state
        if (debug) {
            cout << "cycle check dump 1:" << endl;
            cout << "  endpoints: " << dbg(endpointsClone) << endl;
            cout << "  edgeList: " << edgeCounter << " - " << dbg(edgeList) << endl;
            cout << "  kid endpoints: " << endpsCounter << " - " << dbg(rAllChildEndpoints) << endl;
            cout << "  progress: " << progressCounter << " - v: " << (pV==nullptr ? -1 : pV->vid ) << endl << endl;
        }

        // If we completed the path
        if (pV == nullptr || pV->vid == endpointsClone[progressCounter + 1]) {
            progressCounter += stepsize;
            if (static_cast<unsigned int>(progressCounter) >= endpointsClone.size()) {
                if (edgeCounter == edgeList.size() && endpsCounter == rAllChildEndpoints.size()) {
                    return true;
                } else {
                    if (debug)
                        cout << "ERROR: all endpoints are satisfied, but there are edges or endpoints left" << endl;
                    return false;
                }
            }
            pV = graph.vertices[endpointsClone[progressCounter]].get();
        }

        // Dump the state
        if (debug) {
            cout << "cycle check dump 2:" << endl;
            cout << "  endpoints: " << dbg(endpointsClone) << endl;
            cout << "  edgeList: " << edgeCounter << " - " << dbg(edgeList) << endl;
            cout << "  kid endpoints: " << endpsCounter << " - " << dbg(rAllChildEndpoints) << endl;
            cout << "  progress: " << progressCounter << " - v: " << (pV==nullptr ? -1 : pV->vid ) << endl << endl;
        }

        // Find the next vertex
        bool noBreak = true;
        for (unsigned int i=endpsCounter; i<rAllChildEndpoints.size(); i += stepsize) {
            if (rAllChildEndpoints[i] == pV->vid || rAllChildEndpoints[i + 1] == pV->vid) {
                pV = graph.vertices[rAllChildEndpoints[pV->vid == rAllChildEndpoints[i] ? i + 1 : i]].get();

                int temp = rAllChildEndpoints[endpsCounter];
                rAllChildEndpoints[endpsCounter] = rAllChildEndpoints[i];
                rAllChildEndpoints[i] = temp;
                temp = rAllChildEndpoints[endpsCounter + 1];
                rAllChildEndpoints[endpsCounter + 1] = rAllChildEndpoints[i + 1];
                rAllChildEndpoints[i + 1] = temp;
                if (stepsize >= 3) { // UPDATE THE DEMANDS AS WELL - this is nice, but is it nescessary? TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
                    temp = rAllChildEndpoints[endpsCounter + 2];
                    rAllChildEndpoints[endpsCounter + 2] = rAllChildEndpoints[i + 2];
                    rAllChildEndpoints[i + 2] = temp;
                }

                endpsCounter += stepsize;
                noBreak = false;
                break;
            }
        }
        if (noBreak) {
            // noBreak = true; // Uhh, yeah, doh, if noBreak wasn't true, we wouldn't reach this bit of code right?
            for (unsigned int i=edgeCounter; i<edgeList.size(); ++i) {
                if (edgeList[i]->IsIncidentTo(pV)) {
                    pV = edgeList[i]->Other(*pV);
                    Edge* pTemp = edgeList[edgeCounter];
                    edgeList[edgeCounter] = edgeList[i];
                    edgeList[i] = pTemp;
                    ++edgeCounter;
                    noBreak = false;
                    break;
                }
            }
            if (noBreak) {
                if (debug) {
                    cout << "eps: " << dbg(endpointsClone) << ", edgelist: " << dbg(edgeList) << ", all kid eps: " << dbg(rAllChildEndpoints) << endl;
                    cout << "ERROR, no more endpoints or edges found according to specs" << endl;
                }
                return false;
            }
        }
    }
    cout << "ERROR: The code should not come here (cycleCheck function)." << endl;
    return false;
}

bool inEndpoints(const vector<int>& endpoints, int start, int end, int stepsize /*=2*/) {
    // Return whether or not this combination of endpoints (or reversed order) is already in the endpoints list
    for (unsigned int j=0; j<endpoints.size(); j += stepsize)
        if ((endpoints[j] == start and endpoints[j + 1] == end) || (endpoints[j + 1] == start and endpoints[j] == end))
            return true;
    return false;
}

vector<Edge*> getBagEdges(const Bag& Xi) {
    // Get all edges corresponding to a bag (meaning all the edges of which both endpoints are vertices in this bag)
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
    if (edges.size() > sizeof(int)) {
        cout << "ASSERTION ERROR (getBagEdges): There are more edges in bag X" << Xi.vid << " then there are bits in an int: " << sizeof(int) << endl;
    }
    return edges;
}

vector<Edge*> removeDoubles(const vector<Edge*>& edges, unsigned int length) {
    // Return a vector with each edge only once from the edges-list.
    unsigned int index = 0;
    vector<Edge*> result = vector<Edge*>(length);
    for (unsigned int i=0; i<edges.size(); ++i) {
        bool inList = false;
        Edge* pEdge = edges[i];
        for (unsigned int j=0; j<result.size(); ++j) {
            if (result[j] == nullptr)
                break;
            if (result[j] == pEdge) {
                inList = true;
                break;
            }
        }
        if (!inList) {
            result[index] = pEdge;
            ++index;
        }
    }
    return result;
}

unsigned int toEdgeListBits(const vector<Edge*>& edges, const vector<Edge*>& resultingEdgeList) {
    // Create the bitstring (uint) from the resulting edge list's edges (quite possibly this function will never be used, but still fun to write :]).
    unsigned int result = 0;
    int index = 0;
    for (unsigned int i=0; i<edges.size(); ++i)
        if (edges[i] == resultingEdgeList[index]) {
            result |= 1 << i;
            ++index;
        }
    return result;
}
void addToEdgeListFromBits(const vector<Edge*>& edges, vector<Edge*>* pResultingEdgeList, unsigned int edgeListBits) {
    // Add the edges specified by the bitstring (uint) to the resulting edge list
    for (unsigned int i=0; i<edges.size(); ++i) {
        if ((edgeListBits & 1) == 1)
            pResultingEdgeList->push_back(edges[i]);
        edgeListBits >>= 1;
    }
}
