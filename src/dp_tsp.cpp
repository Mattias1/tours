#include "dp_tsp.h"

#include "graph.h"
#include "treedecomposition.h"
#include "utils.h"
#include <algorithm>
#include <iostream>
#include <limits>
using namespace std;



int DEBUG_COUNTER_TOTAL = 0;
int DEBUG_COUNTER_A = 0;
int DEBUG_COUNTER_B = 0;
int DEBUG_COUNTER_AB = 0;
int DEBUG_COUNTER_NEITHER = 0;



//
// The matching class
//
Matching::Matching()
{}
Matching::Matching(int a, int b)
    :Matching(a, b, -1)
{ }
Matching::Matching(int a, int b, int demand)
    :A(a),
    B(b),
    Demand(demand)
{ }

int Matching::Other(int vid) const {
    // Return the other endpoint in the matching (assuming vid is one of them, if not, return the -1 error code)
    if (vid == this->A)
        return this->B;
    if (vid == this->B)
        return this->A;
    return -1;
}

bool Matching::IsIncidentTo(int vid) const {
    // Return whether or not the vid is one of the endpoints in the matching
    return this->A == vid || this->B == vid;
}

bool Matching::EqualsSortOf(int vid1, int vid2) const {
    // Return whether or not both vids are in this matching
    return (this->A == vid1 && this->B == vid2) || (this->B == vid1 && this->A == vid2);
}

bool Matching::MergeInto(int a, int b, vector<Matching*>& rMatchings, vector<Matching>& rNewMatchingsMemoryManager) {
    // Find out who's in the list (and where)
    int posA = -1;
    int otherA = -1;
    int posB = -1;
    int otherB = -1;
    for (int i=0; i<rMatchings.size(); ++i) {
        if (posA == -1 && rMatchings[i]->IsIncidentTo(a)) {
            posA = i;
            otherA = rMatchings[i]->Other(a);
        }
        else if (posB == -1 && rMatchings[i]->IsIncidentTo(b)) {
            posB = i;
            otherB = rMatchings[i]->Other(b);
        }
        else {
            break;
        }
    }
    // Two cases (1): only one is already in there (in this case the path (matching) is extended) (branch on which one is new)
    if (posA == -1) {
        if (posB == -1) {
            // Third case (3): both are not in here - the matching needs to be added in another way
            ++DEBUG_COUNTER_NEITHER;
            return false;
        }
        ++DEBUG_COUNTER_A;
        rNewMatchingsMemoryManager.push_back(Matching(otherB, a, rMatchings[posB]->Demand));
        rMatchings[posB] = &rNewMatchingsMemoryManager[rNewMatchingsMemoryManager.size() - 1];
    }
    else if (posB == -1) {
        ++DEBUG_COUNTER_B;
        rNewMatchingsMemoryManager.push_back(Matching(otherA, b, rMatchings[posA]->Demand));
        rMatchings[posA] = &rNewMatchingsMemoryManager[rNewMatchingsMemoryManager.size() - 1];
    }
    // or (2): both are already in there (in that case merges the two paths (matchings) together)
    else {
        ++DEBUG_COUNTER_AB;
        rNewMatchingsMemoryManager.push_back(Matching(otherA, b, rMatchings[posA]->Demand));
        rMatchings[posB] = &rNewMatchingsMemoryManager[rNewMatchingsMemoryManager.size() - 1];
        rMatchings.erase(rMatchings.begin() + posB);
    }
    return true;
}

ostream &operator<<(ostream &os, const Matching& m) {
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
        cout << "TspDP ERROR: Probably no root bag present" << endl;
        return vector<Edge*>();
    }

    // Prepare the tables (a hashlist for every bag)
    vector<unique_ptr<unordered_map<string, int>>> hashlists = vector<unique_ptr<unordered_map<string, int>>>(TD.vertices.size());
    for (int i=0; i<hashlists.size(); ++i)
        hashlists[i] = unique_ptr<unordered_map<string, int>>(new unordered_map<string, int>());

    // Start the calculation
    vector<int> degrees = vector<int>(pXroot->vertices.size(), 2);
    string S = fromDegreesEndpoints(degrees, vector<Matching*>());
    int value = tspTable(*TD.getOriginalGraph(), hashlists, S, *pXroot);
    if (consoleOutput)
        cout << "TSP cost: " << value << endl;
    cout << "DEBUG COUNTERS: " << DEBUG_COUNTER_TOTAL << ", " << DEBUG_COUNTER_A << ", " << DEBUG_COUNTER_B << ", " << DEBUG_COUNTER_AB << ", " << DEBUG_COUNTER_NEITHER << endl;

    if (debug) {
        for (int i=0; i<hashlists.size(); ++i) {
            cout << "X" << i << endl;
            for (const auto& iter : *hashlists[i])
                cout << "  " << iter.first << ": " << iter.second << endl;
        }
    }

    // Reconstruct the tour
    if (false && value < numeric_limits<int>::max()) {
        vector<Edge*> tour = removeDoubles(tspReconstruct(*TD.getOriginalGraph(), hashlists, S, *pXroot), TD.getOriginalGraph()->vertices.size());
        // vector<int> allChildEndpointsParameter;
        // cycleCheck(*TD.getOriginalGraph(), vector<int>(), &tour, allChildEndpointsParameter); // Sort the tour - TODO - ERROR, IT DOESN'T ACTUALLY SORT IT
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

int tspTable(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const string& S, const Bag& Xi) {
    // The smallest value such that all vertices below Xi have degree 2 and vertices in Xi have degrees defined by S

    // If we know the value already, just look it up
    const auto& iter = rHashlists[Xi.vid]->find(S);
    if (iter != rHashlists[Xi.vid]->end())
        return iter->second;

    // We don't know this value yet, so we compute it (first get all parameters correct)
    vector<Edge*> Yi = Xi.GetBagEdges();
    vector<int> degrees = toDegrees(S);
    vector<Matching> endpointsMemoryManager = toEndpoints(S);
    vector<Matching*> endpoints = pointerize(endpointsMemoryManager);
    vector<vector<Matching*>> childEndpoints = vector<vector<Matching*>>(Xi.edges.size());
    vector<vector<int>> childDegrees = vector<vector<int>>(Xi.edges.size());
    for (int i=0; i<Xi.edges.size(); ++i) {
        childEndpoints[i] = vector<Matching*>();
        childDegrees[i] = vector<int>(degrees.size(), 0);
    }
    // Recursively find all possible combinations that satisfy the parameter arrays
    (*rHashlists[Xi.vid])[S] = tspRecurse(graph, rHashlists, Xi, Yi, 0, 0, degrees, childDegrees, endpoints, childEndpoints);
    return rHashlists[Xi.vid]->at(S);
}

vector<Edge*> tspReconstruct(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const string& S, const Bag& Xi) {
    // Reconstruct the tsp tour (get a list of all tour edges)
    vector<Edge*> Yi = Xi.GetBagEdges();
    vector<int> degrees = toDegrees(S);
    vector<Matching> endpointsMemoryManager = toEndpoints(S);
    vector<Matching*> endpoints = pointerize(endpointsMemoryManager);
    vector<vector<Matching*>> childEndpoints = vector<vector<Matching*>>(Xi.edges.size());
    vector<vector<int>> childDegrees = vector<vector<int>>(Xi.edges.size());
    for (int i=0; i<Xi.edges.size(); ++i) {
        childEndpoints[i] = vector<Matching*>();
        childDegrees[i] = vector<int>(degrees.size(), 0);
    }
    return tspRecurseVector(graph, rHashlists, Xi, Yi, 0, 0, degrees, childDegrees, endpoints, childEndpoints);
}

int tspChildEvaluation(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge*>& Yi, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<Matching*>& rEndpoints, const vector<vector<Matching*>>& rChildEndpoints, vector<Edge*>* pResultingEdgeList /*=nullptr*/) {
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
    vector<Matching*> allChildEndpoints = flatten(rChildEndpoints);
    int edgeListBits = 0;
    int val = tspEdgeSelect(numeric_limits<int>::max(), 0, graph, Xi, Yi, rTargetDegrees, rEndpoints, allChildEndpoints, &edgeListBits);
    if (pResultingEdgeList != nullptr)
        addToEdgeListFromBits(Yi, pResultingEdgeList, edgeListBits);
    if (0 <= val && val < numeric_limits<int>::max()) { // TODO: why can val ever be < 0??? Why is this first part of the check here??? It is also there in the python version...
        if (debug) {
            cout << dbg("  ", Xi.vertices.size()) << "Local edge selection cost: " << val << ", Yi: " << dbg(Yi) << ", degrees: " << dbg(rTargetDegrees);
            cout << ", endpoints: " << dbg(rEndpoints) << ", edgeList: " << dbg(pResultingEdgeList) << endl;
        }
        for (int k=0; k<rChildDegrees.size(); ++k) {
            const vector<int>& cds = rChildDegrees[k];
            const Bag& Xkid = *dynamic_cast<Bag*>(Xi.edges[k]->Other(Xi));
            if (Xi.getParent() != &Xkid) {
                // Strip off the vertices not in Xkid and add degrees 2 for vertices not in Xi
                vector<int> kidDegrees(Xkid.vertices.size(), 2);
                for (int p=0; p<Xkid.vertices.size(); ++p)
                    for (int q=0; q<Xi.vertices.size(); ++q)
                        if (Xkid.vertices[p] == Xi.vertices[q]) {
                            kidDegrees[p] = cds[q];
                            break;
                        }
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

vector<Edge*> tspLookback(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge*>& Yi, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<Matching*>& rEndpoints, vector<vector<Matching*>>& rChildEndpoints) {
    // This method is the base case for the reconstruct tsp recurse method.
    // bool debug = false;
    vector<Edge*> resultingEdgeList; // This list will be filled with the edges used in Xi
    vector<int> totalDegrees = duplicate(rTargetDegrees);
    for (int j=0; j<rChildDegrees.size(); ++j) {
        const vector<int>& cds = rChildDegrees[j];
        for (int i=0; i<cds.size(); ++i)
            totalDegrees[i] += cds[i];
    }

    string S = fromDegreesEndpoints(totalDegrees, rEndpoints);
    const auto& iter = rHashlists[Xi.vid]->find(S);
    if (iter == rHashlists[Xi.vid]->end())
        return vector<Edge*>();
    int val = iter->second;
    if (val != tspChildEvaluation(graph, rHashlists, Xi, Yi, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints, &resultingEdgeList))
        return vector<Edge*>(); // Side effect above intended to fill the edge list
    // if debug: print('X{} edgelist 1: {}'.format(Xi.vid, resultingEdgeList))
    // So these are indeed the child degrees that we are looking for
    for (int k=0; k<rChildDegrees.size(); ++k) {
        const vector<int>& cds = rChildDegrees[k];

        const Bag& Xkid = *dynamic_cast<Bag*>(Xi.edges[k]->Other(Xi));
        if (Xi.getParent() != &Xkid) {
            // Strip off the vertices not in Xkid and add degrees 2 for vertices not in Xi
            vector<int> kidDegrees(Xkid.vertices.size(), 2);
            for (int p=0; p<Xkid.vertices.size(); ++p)
                for (int q=0; q<Xi.vertices.size(); ++q)
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

int tspRecurse(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge*>& Yi, int i, int j, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<Matching*>& rEndpoints, vector<vector<Matching*>>& rChildEndpoints) {
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
        return tspChildEvaluation(graph, rHashlists, Xi, Yi, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints);
    // Base case: if we can't or didn't want to 'spend' this degree, move on
    if (rTargetDegrees[i] == 0 || j >= Xi.edges.size())
        return tspRecurse(graph, rHashlists, Xi, Yi, i + 1, 0, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints);
    const Bag& Xj = *dynamic_cast<Bag*>(Xi.edges[j]->Other(Xi));
    // Base case: if the current bag (must be child) does not contain the vertex to analyze, try the next (child) bag
    if (Xi.getParent() == Xi.edges[j]->Other(Xi) || !Xj.ContainsVertex(Xi.vertices[i]))
        return tspRecurse(graph, rHashlists, Xi, Yi, i, j + 1, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints);

    // If the current degree is 2, try letting the child manage it
    int result = numeric_limits<int>::max();
    if (rTargetDegrees[i] == 2 and rChildDegrees[j][i] == 0) {
        vector<int> td = duplicate(rTargetDegrees);
        vector<vector<int>> cds = duplicate(rChildDegrees);
        td[i] = 0;
        cds[j][i] = 2;
        result = tspRecurse(graph, rHashlists, Xi, Yi, i + 1, 0, td, cds, rEndpoints, rChildEndpoints);
    }
    // If the current degree is at least 1 (which it is if we get here),
    //   try to combine it (for all other vertices) in a hamiltonian path
    vector<Matching> newMatchingsMemoryManager;
    for (int k=i+1; k<Xi.vertices.size(); ++k) {
        // Stay in {0, 1, 2}
        if (rTargetDegrees[k] < 1 || rChildDegrees[j][k] > 1 || !Xj.ContainsVertex(Xi.vertices[k]))
            continue;
        // Don't add edges twice
        if (inEndpoints(rChildEndpoints[j], Xi.vertices[i]->vid, Xi.vertices[k]->vid))
            continue;
        vector<int> td = duplicate(rTargetDegrees);
        vector<vector<int>> cds = duplicate(rChildDegrees);
        vector<vector<Matching*>> eps = duplicate(rChildEndpoints);
        // Update the degrees
        td[i] -= 1;
        cds[j][i] += 1;
        td[k] -= 1;
        cds[j][k] += 1;
        // Update the endpoints (update when one (or both) of the endpoints are already in the list, otherwise insert)
        bool matchingMightExistAlready = cds[j][i] == 2 || cds[j][k] == 2; // TODO
        if (matchingMightExistAlready) {
            ++DEBUG_COUNTER_TOTAL;
            // So now at least one of the two is not new in the list, so we update it (or them)
            if (!Matching::MergeInto(Xi.vertices[i]->vid, Xi.vertices[k]->vid, eps[j], newMatchingsMemoryManager))
                matchingMightExistAlready = false;
        }
        if (!matchingMightExistAlready) {
            // So now both are new in the list, so we just add (insert) them
            newMatchingsMemoryManager.push_back(Matching(Xi.vertices[i]->vid, Xi.vertices[k]->vid));
            eps[j].push_back(&newMatchingsMemoryManager[newMatchingsMemoryManager.size() - 1]);
        }

        // We may have to try to analyze the same vertex again if it's degree is higher than 1
        result = min(result, tspRecurse(graph, rHashlists, Xi, Yi, i, j, td, cds, rEndpoints, eps));
    }
    // Also, try not assigning this degree to anyone, we (maybe) can solve it inside Xi
    result = min(result, tspRecurse(graph, rHashlists, Xi, Yi, i, j + 1, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints));
    return result;
}

vector<Edge*> tspRecurseVector(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge*>& Yi, int i, int j, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<Matching*>& rEndpoints, vector<vector<Matching*>>& rChildEndpoints) {
    // Select all possible mixes of degrees for all vertices and evaluate them
    //   i = the vertex we currently analyze, j = the child we currently analyze
    //   rTargetDegrees goes from full to empty, rChildDegrees from empty to full, endpoints are the endpoints for each child path
    // bool debug = false;
    // if debug: print('{}{}{}     (X{}: {}, {})   {}|{}'.format('  ' * i, rChildDegrees, '  ' * (len(Xi.vertices) + 8 - i), Xi.vid, i, j, rTargetDegrees, rEndpoints))
    // Final base case.
    if (i >= Xi.vertices.size())
        return tspLookback(graph, rHashlists, Xi, Yi, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints); // BaseF
    // Base case: if we can't or didn't want to 'spend' this degree, move on
    if (rTargetDegrees[i] == 0 || j >= Xi.edges.size())
        return tspRecurseVector(graph, rHashlists, Xi, Yi, i + 1, 0, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints);
    const Bag& Xj = *dynamic_cast<Bag*>(Xi.edges[j]->Other(Xi));
    // Base case: if the current bag (must be child) does not contain the vertex to analyze, try the next (child) bag
    if (Xi.getParent() == Xi.edges[j]->Other(Xi) || !Xj.ContainsVertex(Xi.vertices[i]))
        return tspRecurseVector(graph, rHashlists, Xi, Yi, i, j + 1, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints);

    // If the current degree is 2, try letting the child manage it (it = the i-th vertex in Xi)
    vector<Edge*> result;
    if (rTargetDegrees[i] == 2 and rChildDegrees[j][i] == 0) {
        vector<int> td = duplicate(rTargetDegrees);
        vector<vector<int>> cds = duplicate(rChildDegrees);
        td[i] = 0;
        cds[j][i] = 2;
        result = tspRecurseVector(graph, rHashlists, Xi, Yi, i + 1, 0, td, cds, rEndpoints, rChildEndpoints);
    }
    // If the current degree is at least 1 (which it is if we get here), try to combine it (for all other vertices) in a hamiltonian path
    vector<Matching> newMatchingsMemoryManager;
    for (int k=i+1; k<Xi.vertices.size(); ++k) {
        // Stay in {0, 1, 2} (and make sure child k has the i-th vertex of Xi as well)
        if (rTargetDegrees[k] < 1 || rChildDegrees[j][k] > 1 || !Xj.ContainsVertex(Xi.vertices[k]))
            continue;
        // Don't add edges twice
        if (inEndpoints(rChildEndpoints[j], Xi.vertices[i]->vid, Xi.vertices[k]->vid))
            continue;
        vector<int> td = duplicate(rTargetDegrees);
        vector<vector<int>> cds = duplicate(rChildDegrees);
        vector<vector<Matching*>> eps = duplicate(rChildEndpoints);
        // Update the degrees
        td[i] -= 1;
        cds[j][i] += 1;
        td[k] -= 1;
        cds[j][k] += 1;
        // Update the endpoints (update when one (or both) of the endpoints are already in the list, otherwise insert)
        bool matchingMightExistAlready = cds[j][i] == 2 || cds[j][k] == 2;
        if (matchingMightExistAlready) {
            // So now at least one of the two is not new in the list, so we update it (or them)
            if (!Matching::MergeInto(Xi.vertices[i]->vid, Xi.vertices[k]->vid, eps[j], newMatchingsMemoryManager))
              matchingMightExistAlready = false;
        }
        if (!matchingMightExistAlready) {
            // So now both are new in the list, so we just add (insert) them
            Matching matching = Matching(Xi.vertices[i]->vid, Xi.vertices[k]->vid);
            newMatchingsMemoryManager.push_back(matching);
            eps[j].push_back(&matching);
        }

        // We may have to try to analyze the same vertex again if it's degree is higher than 1
        pushBackList(&result, tspRecurseVector(graph, rHashlists, Xi, Yi, i, j, td, cds, rEndpoints, eps));
    }
    // Also, try not assigning this degree to anyone, we (maybe) can solve it inside Xi
    pushBackList(&result, tspRecurseVector(graph, rHashlists, Xi, Yi, i, j + 1, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints));
    return result;
}

// Todo: use the minimum to abort early??? (is possible for leaf case, but perhaps not for normal bag case
int tspEdgeSelect(int minimum, int index, const Graph& graph, const Bag& Xi, const vector<Edge*>& Yi, const vector<int>& degrees, vector<Matching*>& rEndpoints, vector<Matching*>& rAllChildEndpoints, int* pEdgeListBits /*=nullptr*/) {
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
    if (assertCounter != 0 && assertCounter != 2)
        cout << "ASSERTION ERROR - the assertCounter is not 0 or 2." << endl;

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
vector<int> toDegrees(const string& S) {
    // From a string representation to a list of degrees
    return splitInt(split(S, '|')[0], ',');
}

vector<Matching> toEndpoints(const string& S) {
    // From a string representation to a list of edges
    vector<Matching> result;
    vector<string> tempSplit = split(S, '|');
    if (tempSplit.size() < 2)
        return result;

    vector<int> tempInts = splitInt(tempSplit[1], ',');
    for (int i=0; i<tempInts.size(); i+=3)
        result.push_back(Matching(tempInts[i], tempInts[i + 1], tempInts[i + 2]));
    return result;
}

string fromDegreesEndpoints(const vector<int>& degrees, const vector<Matching*>& endpoints) {
    // From a list of degrees and endpoints to a string representation
    return join(degrees, ',') + '|' + join(endpoints, ',');
}

bool cycleCheck(const Graph& graph, const vector<Matching*>& endpoints, vector<Edge*>* pEdgeList, vector<Matching*>& rAllChildEndpoints) {
    // This method returns whether or not the given edge list and all child endpoints provide a set of paths
    // satisfying the endpoints and sorts the edge list in place.
    bool debug = false;

    // Inits
    int progressCounter = -1;
    int edgeCounter = 0;
    int endpsCounter = 0;
    Vertex* pV = nullptr;
    int targetVid = 0;
    vector<Matching*> endpointsClone = duplicate(endpoints);

    vector<Edge*> edgeList;
    if (pEdgeList == nullptr)
        edgeList = vector<Edge*>(); // Note that pEdgeList still is null, so I should use the variable edgeList everywhere in this method.
    else
        edgeList = *pEdgeList; // If I understand C++ well, this invokes the copy constructor. That'd be good.

    // Special init case for the root bag.
    Matching extraEndpointMemoryManager;
    if (endpointsClone.size() == 0) {
        if (rAllChildEndpoints.size() > 0) {
            endpointsClone.push_back(rAllChildEndpoints[0]);
            endpsCounter += 1;
        }
        else if (edgeList.size() > 0) {
            extraEndpointMemoryManager = Matching(edgeList[0]->pA->vid, edgeList[0]->pB->vid);
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

                Matching* pTemp = rAllChildEndpoints[endpsCounter];
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
    cout << "ERROR: The code should not come here (cycleCheck function)." << endl;
    return false;
}

bool inEndpoints(const vector<Matching*>& endpoints, int vid1, int vid2) {
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
