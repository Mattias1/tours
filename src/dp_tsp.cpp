#include "dp_tsp.h"

#include "graph.h"
#include "treedecomposition.h"
#include "utils.h"
#include <algorithm>
#include <iostream>
#include <limits>
#include <assert.h>
using namespace std;

//
// The matching-edge class
//
MatchingEdge::MatchingEdge()
{}
MatchingEdge::MatchingEdge(int a, int b)
    :MatchingEdge(a, b, -1)
{ }
MatchingEdge::MatchingEdge(int a, int b, int demand)
    :A(a),
    B(b),
    Demand(demand)
{ }

int MatchingEdge::Other(int vid) const {
    // Return the other endpoint in the matching (assuming vid is one of them, if not, return the -1 error code)
    if (vid == this->A)
        return this->B;
    if (vid == this->B)
        return this->A;
    return -1;
}

bool MatchingEdge::IsIncidentTo(int vid) const {
    // Return whether or not the vid is one of the endpoints in the matching
    return this->A == vid || this->B == vid;
}

bool MatchingEdge::EqualsSortOf(int vid1, int vid2) const {
    // Return whether or not both vids are in this matching
    return (this->A == vid1 && this->B == vid2) || (this->B == vid1 && this->A == vid2);
}

bool MatchingEdge::MergeInto(int a, int b, vector<MatchingEdge*>& rMatching, vector<unique_ptr<MatchingEdge>>& rMatchingsMemoryManager, bool dontMergeDepot /*=false*/) {
    // Find out who's in the list (and where)
    int posA = -1;
    int otherA = -1;
    int posB = -1;
    int otherB = -1;
    for (int i=0; i<rMatching.size(); ++i) {
        if (posA == -1 && !(dontMergeDepot && isDepot(a)) && rMatching[i]->IsIncidentTo(a)) {
            posA = i;
            otherA = rMatching[i]->Other(a);
        }
        else if (posB == -1 && !(dontMergeDepot && isDepot(b)) && rMatching[i]->IsIncidentTo(b)) {
            posB = i;
            otherB = rMatching[i]->Other(b);
        }
        else {
            break;
        }
    }
    // There are three cases.
    // Case 1: only one is already in there (in this case the path (matching) is extended) (branch on which one is new)
    if (posA == -1) {
        if (posB == -1) {
            // Case 3: both are not in here - the matching needs to be added in another way (in another part of the code: the recurse functions)
            return false;
        }
        rMatchingsMemoryManager.push_back(unique_ptr<MatchingEdge>(new MatchingEdge(otherB, a, rMatching[posB]->Demand)));
        rMatching[posB] = rMatchingsMemoryManager.back().get();
    }
    else if (posB == -1) {
        rMatchingsMemoryManager.push_back(unique_ptr<MatchingEdge>(new MatchingEdge(otherA, b, rMatching[posA]->Demand)));
        rMatching[posA] = rMatchingsMemoryManager.back().get();
    }
    // Case 2: both are already in there (in that case merges the two paths (matchings) together)
    else {
        rMatchingsMemoryManager.push_back(unique_ptr<MatchingEdge>(new MatchingEdge(otherA, b, rMatching[posA]->Demand)));
        rMatching[posB] = rMatchingsMemoryManager.back().get();
        rMatching.erase(rMatching.begin() + posB);
    }
    return true;
}

ostream &operator<<(ostream &os, const MatchingEdge& m) {
    if (m.Demand >= 0)
        return os << m.A << "-" << m.B << ": " << m.Demand;
    return os << m.A << "-" << m.B;
}



//
//  The core functions used to calculate the tour for the TSP problem.
//
vector<Edge*> tspDP(const TreeDecomposition& TD, bool consoleOutput /*=true*/) {
    // Compute the smallest tour using DP on a tree decomposition.
    bool debug = false;

    // Make sure there is a proper graph and decomposition
    const Bag* pXroot = TD.getRoot();
    if (TD.vertices.size() < 1 || pXroot == nullptr || TD.getOriginalGraph()->vertices.size() < 1) {
        cout << "ERROR (TSP-DP): Probably no root bag present" << endl;
        return vector<Edge*>();
    }

    // Start the calculation
    unordered_map<string, int> hashlist;
    vector<int> degrees = vector<int>(pXroot->vertices.size(), 2);
    string S = toTableEntry(*pXroot, degrees, vector<MatchingEdge*>());
    int value = tspTable(*TD.getOriginalGraph(), hashlist, S, *pXroot);
    if (consoleOutput)
        cout << "TSP cost: " << value << endl;

    if (debug) {
        for (const auto& iter : hashlist)
            cout << "  " << iter.first << ": " << iter.second << endl;
    }

    // Reconstruct the tour
    if (false && value < numeric_limits<int>::max()) { // REMOVE THE FALSE HERE TO MAKE IT CALCULATE THE FOUND TOUR
        vector<Edge*> tour = removeDoubles(tspReconstruct(*TD.getOriginalGraph(), hashlist, S, *pXroot), TD.getOriginalGraph()->vertices.size());
        if (debug) {
            cout << "\nDP-TSP:\n  Length: " << value << "\n  Tour: ";
            for (int i=0; i<tour.size(); ++i)
                cout << *tour[i] << ", ";
            cout << endl;
        }
        return tour;
    }
    return vector<Edge*>();
}

int tspTable(const Graph& graph, unordered_map<string, int>& rHashlist, const string& S, const Bag& Xi) {
    // The smallest value such that all vertices below Xi have degree 2 and vertices in Xi have degrees defined by S

    // If we know the value already, just look it up
    const auto& iter = rHashlist.find(S);
    if (iter != rHashlist.end())
        return iter->second;

    // We don't know this value yet, so we compute it (first get all parameters correct)
    vector<Edge*> Yi = Xi.GetBagEdges();
    vector<int> degrees = tableEntryToDegrees(S);
    vector<MatchingEdge> endpointsMemoryManager = tableEntryToEndpoints(S);
    vector<MatchingEdge*> endpoints = pointerize(endpointsMemoryManager);
    vector<vector<MatchingEdge*>> childEndpoints = vector<vector<MatchingEdge*>>(Xi.edges.size());
    vector<vector<int>> childDegrees = vector<vector<int>>(Xi.edges.size());
    for (int i=0; i<Xi.edges.size(); ++i) {
        childEndpoints[i] = vector<MatchingEdge*>();
        childDegrees[i] = vector<int>(degrees.size(), 0);
    }
    // Recursively find all possible combinations that satisfy the parameter arrays
    rHashlist[S] = tspRecurse(graph, rHashlist, Xi, Yi, 0, 0, degrees, childDegrees, endpoints, childEndpoints);
    return rHashlist.at(S);
}

vector<Edge*> tspReconstruct(const Graph& graph, unordered_map<string, int>& rHashlist, const string& S, const Bag& Xi) {
    // Reconstruct the tsp tour (get a list of all tour edges)
    vector<Edge*> Yi = Xi.GetBagEdges();
    vector<int> degrees = tableEntryToDegrees(S);
    vector<MatchingEdge> endpointsMemoryManager = tableEntryToEndpoints(S);
    vector<MatchingEdge*> endpoints = pointerize(endpointsMemoryManager);
    vector<vector<MatchingEdge*>> childEndpoints = vector<vector<MatchingEdge*>>(Xi.edges.size());
    vector<vector<int>> childDegrees = vector<vector<int>>(Xi.edges.size());
    for (int i=0; i<Xi.edges.size(); ++i) {
        childEndpoints[i] = vector<MatchingEdge*>();
        childDegrees[i] = vector<int>(degrees.size(), 0);
    }
    return tspRecurseVector(graph, rHashlist, Xi, Yi, 0, 0, degrees, childDegrees, endpoints, childEndpoints);
}

int tspRecurse(const Graph& graph, unordered_map<string, int>& rHashlist, const Bag& Xi, const vector<Edge*>& Yi, int i, int j, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<MatchingEdge*>& rEndpoints, vector<vector<MatchingEdge*>>& rChildEndpoints) {
    // Select all possible mixes of degrees for all vertices and evaluate them
    //   i = the vertex we currently analyze, j = the child-bag we currently analyze
    //   rTargetDegrees goes from full to empty, rChildDegrees from empty to full, endpoints are the endpoints for each child path
    bool debug = false;
    if (debug) {
        // tree-of-childDegrees          (Xi: i, j)   targetDegrees|endpoints
        cout << dbg("  ", i) << dbg(rChildDegrees) << dbg("  ", Xi.vertices.size() + 9 - i);
        cout << "(X" << Xi.vid << ": " << i << ", " << j << ")   " << dbg(rTargetDegrees) << "|" << dbg(rEndpoints) << endl;
    }

    // Final base case.
    if (i >= Xi.vertices.size())
        return tspChildEvaluation(graph, rHashlist, Xi, Yi, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints);
    // Base case: if we can't or didn't want to 'spend' this degree, move on
    if (rTargetDegrees[i] == 0 || j >= Xi.edges.size())
        return tspRecurse(graph, rHashlist, Xi, Yi, i + 1, 0, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints);
    const Bag& Xj = *dynamic_cast<Bag*>(Xi.edges[j]->Other(Xi));
    // Base case: if the current bag (must be child) does not contain the vertex to analyze, try the next (child) bag
    if (Xi.getParent() == Xi.edges[j]->Other(Xi) || !Xj.ContainsVertex(Xi.vertices[i]))
        return tspRecurse(graph, rHashlist, Xi, Yi, i, j + 1, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints);

    int result = numeric_limits<int>::max();
    // If the current degree is 2, try letting the child manage it
    if (rTargetDegrees[i] == 2 and rChildDegrees[j][i] == 0) {
        vector<int> td = duplicate(rTargetDegrees);
        vector<vector<int>> cds = duplicate(rChildDegrees);
        td[i] = 0;
        cds[j][i] = 2;
        result = tspRecurse(graph, rHashlist, Xi, Yi, i + 1, 0, td, cds, rEndpoints, rChildEndpoints);
    }
    // If the current degree is at least 1 (which it is if we get here),
    //   try to combine it (for all other vertices) in a hamiltonian path
    vector<unique_ptr<MatchingEdge>> matchingsMemoryManager;
    for (int k=i+1; k<Xi.vertices.size(); ++k) {
        // Stay in {0, 1, 2}
        if (rTargetDegrees[k] < 1 || rChildDegrees[j][k] > 1 || !Xj.ContainsVertex(Xi.vertices[k]))
            continue;
        // Don't add edges twice
        if (inEndpoints(rChildEndpoints[j], Xi.vertices[i]->vid, Xi.vertices[k]->vid))
            continue;
        vector<int> td = duplicate(rTargetDegrees);
        vector<vector<int>> cds = duplicate(rChildDegrees);
        vector<vector<MatchingEdge*>> eps = duplicate(rChildEndpoints);
        // Update the degrees
        td[i] -= 1;
        cds[j][i] += 1;
        td[k] -= 1;
        cds[j][k] += 1;
        // Update the endpoints (update when one (or both) of the endpoints are already in the list, otherwise insert)
        bool edgeMightExistAlready = cds[j][i] == 2 || cds[j][k] == 2;
        if (edgeMightExistAlready) {
            // So now at least one of the two is not new in the list, so we update it (or them)
            if (!MatchingEdge::MergeInto(Xi.vertices[i]->vid, Xi.vertices[k]->vid, eps[j], matchingsMemoryManager))
                edgeMightExistAlready = false;
        }
        if (!edgeMightExistAlready) {
            // So now both are new in the list, so we just add (insert) them
            matchingsMemoryManager.push_back(unique_ptr<MatchingEdge>(new MatchingEdge(Xi.vertices[i]->vid, Xi.vertices[k]->vid)));
            eps[j].push_back(matchingsMemoryManager.back().get());
        }

        // We may have to try to analyze the same vertex again if it's degree is higher than 1
        result = min(result, tspRecurse(graph, rHashlist, Xi, Yi, i, j + 1, td, cds, rEndpoints, eps));
    }
    // Also, try not assigning this degree right now, it might be solved for for another child, or we (maybe) can solve it inside Xi
    result = min(result, tspRecurse(graph, rHashlist, Xi, Yi, i, j + 1, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints));
    return result;
}

vector<Edge*> tspRecurseVector(const Graph& graph, unordered_map<string, int>& rHashlist, const Bag& Xi, const vector<Edge*>& Yi, int i, int j, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<MatchingEdge*>& rEndpoints, vector<vector<MatchingEdge*>>& rChildEndpoints) {
    // Select all possible mixes of degrees for all vertices and evaluate them
    //   i = the vertex we currently analyze, j = the child we currently analyze
    //   rTargetDegrees goes from full to empty, rChildDegrees from empty to full, endpoints are the endpoints for each child path
    // bool debug = false;
    // if debug: print('{}{}{}     (X{}: {}, {})   {}|{}'.format('  ' * i, rChildDegrees, '  ' * (len(Xi.vertices) + 8 - i), Xi.vid, i, j, rTargetDegrees, rEndpoints))
    // Final base case.
    if (i >= Xi.vertices.size())
        return tspLookback(graph, rHashlist, Xi, Yi, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints); // BaseF
    // Base case: if we can't or didn't want to 'spend' this degree, move on
    if (rTargetDegrees[i] == 0 || j >= Xi.edges.size())
        return tspRecurseVector(graph, rHashlist, Xi, Yi, i + 1, 0, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints);
    const Bag& Xj = *dynamic_cast<Bag*>(Xi.edges[j]->Other(Xi));
    // Base case: if the current bag (must be child) does not contain the vertex to analyze, try the next (child) bag
    if (Xi.getParent() == Xi.edges[j]->Other(Xi) || !Xj.ContainsVertex(Xi.vertices[i]))
        return tspRecurseVector(graph, rHashlist, Xi, Yi, i, j + 1, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints);

    // If the current degree is 2, try letting the child manage it (it = the i-th vertex in Xi)
    vector<Edge*> result;
    if (rTargetDegrees[i] == 2 and rChildDegrees[j][i] == 0) {
        vector<int> td = duplicate(rTargetDegrees);
        vector<vector<int>> cds = duplicate(rChildDegrees);
        td[i] = 0;
        cds[j][i] = 2;
        result = tspRecurseVector(graph, rHashlist, Xi, Yi, i + 1, 0, td, cds, rEndpoints, rChildEndpoints);
    }
    // If the current degree is at least 1 (which it is if we get here), try to combine it (for all other vertices) in a hamiltonian path
    vector<unique_ptr<MatchingEdge>> matchingsMemoryManager;
    for (int k=i+1; k<Xi.vertices.size(); ++k) {
        // Stay in {0, 1, 2} (and make sure child k has the i-th vertex of Xi as well)
        if (rTargetDegrees[k] < 1 || rChildDegrees[j][k] > 1 || !Xj.ContainsVertex(Xi.vertices[k]))
            continue;
        // Don't add edges twice
        if (inEndpoints(rChildEndpoints[j], Xi.vertices[i]->vid, Xi.vertices[k]->vid))
            continue;
        vector<int> td = duplicate(rTargetDegrees);
        vector<vector<int>> cds = duplicate(rChildDegrees);
        vector<vector<MatchingEdge*>> eps = duplicate(rChildEndpoints);
        // Update the degrees
        td[i] -= 1;
        cds[j][i] += 1;
        td[k] -= 1;
        cds[j][k] += 1;
        // Update the endpoints (update when one (or both) of the endpoints are already in the list, otherwise insert)
        bool edgeMightExistAlready = cds[j][i] == 2 || cds[j][k] == 2;
        if (edgeMightExistAlready) {
            // So now at least one of the two is not new in the list, so we update it (or them)
            if (!MatchingEdge::MergeInto(Xi.vertices[i]->vid, Xi.vertices[k]->vid, eps[j], matchingsMemoryManager))
              edgeMightExistAlready = false;
        }
        if (!edgeMightExistAlready) {
            // So now both are new in the list, so we just add (insert) them
            matchingsMemoryManager.push_back(unique_ptr<MatchingEdge>(new MatchingEdge(Xi.vertices[i]->vid, Xi.vertices[k]->vid)));
            eps[j].push_back(matchingsMemoryManager.back().get());
        }

        // We may have to try to analyze the same vertex again if it's degree is higher than 1
        pushBackList(&result, tspRecurseVector(graph, rHashlist, Xi, Yi, i, j, td, cds, rEndpoints, eps));
    }
    // Also, try not assigning this degree to anyone, we (maybe) can solve it inside Xi
    pushBackList(&result, tspRecurseVector(graph, rHashlist, Xi, Yi, i, j + 1, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints));
    return result;
}

int tspChildEvaluation(const Graph& graph, unordered_map<string, int>& rHashlist, const Bag& Xi, const vector<Edge*>& Yi, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<MatchingEdge*>& rEndpoints, const vector<vector<MatchingEdge*>>& rChildEndpoints, vector<Edge*>* pResultingEdgeList /*=nullptr*/) {
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
    vector<MatchingEdge*> allChildEndpoints = flatten(rChildEndpoints);
    int edgeListBits = 0;
    int val = tspEdgeSelect(numeric_limits<int>::max(), 0, graph, Xi, Yi, rTargetDegrees, rEndpoints, allChildEndpoints, &edgeListBits);
    if (pResultingEdgeList != nullptr)
        addToEdgeListFromBits(Yi, pResultingEdgeList, edgeListBits);
    if (val < numeric_limits<int>::max()) {
        if (debug) {
            cout << dbg("  ", Xi.vertices.size()) << "Local edge selection cost: " << val << ", Yi: " << dbg(Yi) << ", degrees: " << dbg(rTargetDegrees);
            cout << ", endpoints: " << dbg(rEndpoints) << ", edgeList: " << dbg(pResultingEdgeList) << endl;
        }
        for (int j=0; j<rChildDegrees.size(); ++j) {
            const vector<int>& cds = rChildDegrees[j];
            const Bag& Xkid = *dynamic_cast<Bag*>(Xi.edges[j]->Other(Xi));
            if (Xi.getParent() != &Xkid) {
                // Strip off the vertices not in Xkid and add degrees 2 for vertices not in Xi
                vector<int> kidDegrees(Xkid.vertices.size(), 2);
                for (int p=0; p<Xkid.vertices.size(); ++p)
                    for (int q=0; q<Xi.vertices.size(); ++q)
                        if (Xkid.vertices[p] == Xi.vertices[q]) {
                            kidDegrees[p] = cds[q];
                            break;
                        }
                string S = toTableEntry(Xkid, kidDegrees, rChildEndpoints[j]);
                if (debug) {
                    cout << dbg("  ", Xi.vertices.size()) << "child A: " << val << ", cds: " << dbg(cds) << ", degrees: " << dbg(kidDegrees);
                    cout << ", endpoints: " << dbg(rChildEndpoints[j]) << endl;
                }
                // Add to that base cost the cost of hamiltonian paths nescessary to satisfy the degrees.
                int tableVal = tspTable(graph, rHashlist, S, Xkid);
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

vector<Edge*> tspLookback(const Graph& graph, unordered_map<string, int>& rHashlist, const Bag& Xi, const vector<Edge*>& Yi, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<MatchingEdge*>& rEndpoints, vector<vector<MatchingEdge*>>& rChildEndpoints) {
    // This method is the base case for the reconstruct tsp recurse method.
    // bool debug = false;
    vector<Edge*> resultingEdgeList; // This list will be filled with the edges used in Xi
    vector<int> totalDegrees = duplicate(rTargetDegrees);
    for (int j=0; j<rChildDegrees.size(); ++j) {
        const vector<int>& cds = rChildDegrees[j];
        for (int i=0; i<cds.size(); ++i)
            totalDegrees[i] += cds[i];
    }

    string S = toTableEntry(Xi, totalDegrees, rEndpoints);
    const auto& iter = rHashlist.find(S);
    if (iter == rHashlist.end())
        return vector<Edge*>();
    int val = iter->second;
    if (val != tspChildEvaluation(graph, rHashlist, Xi, Yi, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints, &resultingEdgeList))
        return vector<Edge*>(); // Side effect above intended to fill the edge list
    // if debug: print('X{} edgelist 1: {}'.format(Xi.vid, resultingEdgeList))
    // So these are indeed the child degrees that we are looking for
    for (int j=0; j<rChildDegrees.size(); ++j) {
        const vector<int>& cds = rChildDegrees[j];

        const Bag& Xkid = *dynamic_cast<Bag*>(Xi.edges[j]->Other(Xi));
        if (Xi.getParent() != &Xkid) {
            // Strip off the vertices not in Xkid and add degrees 2 for vertices not in Xi
            vector<int> kidDegrees(Xkid.vertices.size(), 2);
            for (int p=0; p<Xkid.vertices.size(); ++p)
                for (int q=0; q<Xi.vertices.size(); ++q)
                    if (Xkid.vertices[p] == Xi.vertices[q])
                        kidDegrees[p] = cds[q];
            string S = toTableEntry(Xkid, kidDegrees, rChildEndpoints[j]);
            // We already got the resultingEdgeList for Xi, now add the REL for all the children
            pushBackList(&resultingEdgeList, tspReconstruct(graph, rHashlist, S, Xkid));
            // print('test 2 edgelist: {}'.format(resultingEdgeList))
        }
    }
    // if debug: print('X{} edgelist 3: {}'.format(Xi.vid, resultingEdgeList))
    return resultingEdgeList;
}

// Todo: use the minimum to abort early??? (is possible for leaf case, but perhaps not for normal bag case
int tspEdgeSelect(int minimum, int index, const Graph& graph, const Bag& Xi, const vector<Edge*>& Yi, const vector<int>& degrees, vector<MatchingEdge*>& rEndpoints, vector<MatchingEdge*>& rAllChildEndpoints, int* pEdgeListBits /*=nullptr*/) {
    // Calculate the smallest cost to satisfy the degrees target using only using edges >= the index
    bool debug = false;

    // Base case 1: the degrees are all zero, so we succeeded as we don't need to add any more edges
    bool satisfied = true;
    for (int i=0; i<degrees.size(); ++i) {
        if (degrees[i] != 0) {
            satisfied = false;
            break;
        }
    }
    if (satisfied) {
        // So we have chosen all our edges and satisfied the targets - now make sure there is no cycle (unless root)
        vector<Edge*> edgeList;
        addToEdgeListFromBits(Yi, &edgeList, *pEdgeListBits);
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
    if (index >= Yi.size()) {
        if (debug)
            cout << "Edge select (" << index << "): no more edges to add" << endl;
        return numeric_limits<int>::max();
    }

    // Base case 3: one of the degrees is < 1, so we added too many vertices, so we failed [with side effect]
    Edge* pEdge = Yi[index];
    vector<int> deg = duplicate(degrees);
    int assertCounter = 0;
    for (int i=0; i<deg.size(); ++i) {
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
    assert(assertCounter == 0 || assertCounter == 2);

    // Try both to take the edge and not to take the edge
    if (debug)
        cout << "Edge select (" << index << "), degrees: " << dbg(degrees) << endl;
    int tempEL1, tempEL2;
    if (pEdgeListBits != nullptr) {
        tempEL1 = *pEdgeListBits;
        tempEL2 = tempEL1;
    }
    tempEL1 |= 1 << index;
    int val = tspEdgeSelect(minimum - pEdge->Cost, index + 1, graph, Xi, Yi, deg, rEndpoints, rAllChildEndpoints, &tempEL1);
    if (val < numeric_limits<int>::max())
        minimum = min(minimum, pEdge->Cost + val);
    val = tspEdgeSelect(minimum, index + 1, graph, Xi, Yi, degrees, rEndpoints, rAllChildEndpoints, &tempEL2);
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
vector<int> tableEntryToDegrees(const string& S) {
    // From a string representation to a list of degrees
    return splitInt(split(S, '|')[1], ',');
}

vector<MatchingEdge> tableEntryToEndpoints(const string& S) {
    // From a string representation to a list of edges
    vector<MatchingEdge> result;
    vector<string> tempSplit = split(S, '|');
    if (tempSplit.size() < 3)
        return result;

    vector<int> tempInts = splitInt(tempSplit[2], ',');
    for (int i=0; i<tempInts.size(); i+=3)
        result.push_back(MatchingEdge(tempInts[i], tempInts[i + 1], tempInts[i + 2]));
    return result;
}

string toTableEntry(const Bag& Xi, const vector<int>& degrees, const vector<MatchingEdge*>& endpoints) {
    // From a list of degrees and endpoints to a string representation
    return 'X' + to_string(Xi.vid) + '|' + join(degrees, ',') + '|' + join(endpoints, ',');
}
string toTableEntry(const Bag& Xi, const vector<int>& degrees, const vector<MatchingEdge>& endpoints) {
    // From a list of degrees and endpoints to a string representation
    return 'X' + to_string(Xi.vid) + '|' + join(degrees, ',') + '|' + join(endpoints, ',');
}

bool cycleCheck(const Graph& graph, const vector<MatchingEdge*>& endpoints, vector<Edge*>* pEdgeList, vector<MatchingEdge*>& rAllChildEndpoints) {
    // This method returns whether or not the given edge list and all child endpoints provide a set of paths
    // satisfying the endpoints and sorts the edge list in place (true if there is NO cycle).
    bool debug = false;

    // Inits
    int progressCounter = -1;
    int edgeCounter = 0;
    int endpsCounter = 0;
    Vertex* pV = nullptr;
    int targetVid = 0;
    vector<MatchingEdge*> endpointsClone = duplicate(endpoints);

    vector<Edge*> edgeList;
    if (pEdgeList == nullptr)
        edgeList = vector<Edge*>(); // Note that pEdgeList still is null, so I should use the variable edgeList everywhere in this method.
    else
        edgeList = *pEdgeList; // If I understand C++ well, this invokes the copy constructor. That'd be good.

    // Special init case for the root bag.
    MatchingEdge extraEndpointMemoryManager;
    if (endpointsClone.size() == 0) {
        if (rAllChildEndpoints.size() > 0) {
            endpointsClone.push_back(rAllChildEndpoints[0]);
            endpsCounter += 1;
        }
        else if (edgeList.size() > 0) {
            extraEndpointMemoryManager = MatchingEdge(edgeList[0]->pA->vid, edgeList[0]->pB->vid);
            endpointsClone.push_back(&extraEndpointMemoryManager);
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
        if (pV == nullptr || pV->vid == targetVid) {
            ++progressCounter;
            if (progressCounter >= endpointsClone.size()) {
                if (edgeCounter == edgeList.size() && endpsCounter == rAllChildEndpoints.size()) {
                    return true;
                } else {
                    if (debug)
                        cout << "ERROR: all endpoints are satisfied, but there are edges or endpoints left" << endl;
                    return false;
                }
            }
            pV = graph.vertices[endpointsClone[progressCounter]->A].get();
            targetVid = endpointsClone[progressCounter]->B;
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
        for (int i=endpsCounter; i<rAllChildEndpoints.size(); ++i) {
            if (rAllChildEndpoints[i]->IsIncidentTo(pV->vid)) {
                pV = graph.vertices[rAllChildEndpoints[i]->Other(pV->vid)].get();

                MatchingEdge* pTemp = rAllChildEndpoints[endpsCounter];
                rAllChildEndpoints[endpsCounter] = rAllChildEndpoints[i];
                rAllChildEndpoints[i] = pTemp;

                ++endpsCounter;
                noBreak = false;
                break;
            }
        }
        if (noBreak) {
            // noBreak = true; // Uhh, yeah, doh, if noBreak wasn't true, we wouldn't reach this bit of code right?
            for (int i=edgeCounter; i<edgeList.size(); ++i) {
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
    cout << "ASSERTION ERROR: The code should not come here (cycleCheck function)." << endl;
    assert(false);
}

bool inEndpoints(const vector<MatchingEdge*>& endpoints, int vid1, int vid2) {
    // Return whether or not this combination of endpoints (or reversed order) is already in the endpoints list
    for (int j=0; j<endpoints.size(); ++j)
        if (endpoints[j]->EqualsSortOf(vid1, vid2))
            return true;
    return false;
}

vector<Edge*> removeDoubles(const vector<Edge*>& edges, int length) {
    // Return a vector with each edge only once from the edges-list.
    int index = 0;
    vector<Edge*> result = vector<Edge*>(length);
    for (int i=0; i<edges.size(); ++i) {
        bool inList = false;
        Edge* pEdge = edges[i];
        for (int j=0; j<result.size(); ++j) {
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

int toEdgeListBits(const vector<Edge*>& Yi, const vector<Edge*>& resultingEdgeList) {
    // Create the bitstring (uint) from the resulting edge list's edges (quite possibly this function will never be used, but still fun to write :]).
    int result = 0;
    int index = 0;
    for (int i=0; i<Yi.size(); ++i)
        if (Yi[i] == resultingEdgeList[index]) {
            result |= 1 << i;
            ++index;
        }
    return result;
}
void addToEdgeListFromBits(const vector<Edge*>& Yi, vector<Edge*>* pResultingEdgeList, int edgeListBits) {
    // Add the edges specified by the bitstring (uint) to the resulting edge list
    for (int i=0; i<Yi.size(); ++i) {
        if ((edgeListBits & 1) == 1)
            pResultingEdgeList->push_back(Yi[i]);
        edgeListBits >>= 1;
    }
}
