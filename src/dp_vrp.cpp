#include "dp_vrp.h"

#include "graph.h"
#include "treedecomposition.h"
#include "utils.h"
#include <algorithm>
#include <iostream>
#include <limits>
using namespace std;

// TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
// TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
//
// TODO: Make sure the depot vertex is in every bag
//
// TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
// TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO


//
//  The core functions used to calculate the tour for the VRP problem.
//
vector<vector<Edge*>> vrpDP(const TreeDecomposition& TD, bool consoleOutput /*=true*/) {
    // Compute the smallest tour using DP on a tree decomposition.
    bool debug = false;

    // Make sure there is a proper graph and decomposition
    const Bag* pXroot = TD.getRoot();
    if (TD.vertices.size() < 1 || pXroot == nullptr || TD.getOriginalGraph()->vertices.size() < 1)
        return vector<vector<Edge*>>();

    // Prepare the tables (a hashlist for every bag)
    vector<unique_ptr<unordered_map<string, int>>> hashlists = vector<unique_ptr<unordered_map<string, int>>>(TD.vertices.size());
    for (unsigned int i=0; i<hashlists.size(); ++i)
        hashlists[i] = unique_ptr<unordered_map<string, int>>(new unordered_map<string, int>());

    // Start the calculation
    vector<int> degrees = vector<int>(pXroot->vertices.size(), 2);
    string S = fromDegreesEndpoints(degrees, vector<int>());
    int value = vrpTable(*TD.getOriginalGraph(), hashlists, S, *pXroot);
    if (consoleOutput)
        cout << "VRP cost: " << value << endl;
    if (debug) {
        for (unsigned int i=0; i<hashlists.size(); ++i) {
            cout << "X" << i << endl;
            for (const auto& iter : *hashlists[i])
                cout << "  " << iter.first << ": " << iter.second << endl;
        }
    }

    // Reconstruct the tour
    if (value < numeric_limits<int>::max()) {
        vector<vector<Edge*>> tours = vrpReconstruct(*TD.getOriginalGraph(), hashlists, S, *pXroot);
        for (unsigned int i=0; i<tours.size(); ++i)
            tours[i] = removeDoubles(tours[i], TD.getOriginalGraph()->vertices.size());
        // vector<int> allChildEndpointsParameter;
        // cycleCheck(*TD.getOriginalGraph(), vector<int>(), &tour, allChildEndpointsParameter, 3); // Sort the tour - TODO - ERROR, IT DOESN'T ACTUALLY SORT IT
        if (debug) {
            cout << "\nDP-TSP:\n  Length: " << value << endl;
            for (unsigned int i=0; i<tours.size(); ++i) {
                cout << "  Tour #" << i << ": ";
                for (unsigned int j=0; j<tours[i].size(); ++j)
                    cout << *tours[i][j] << ", ";
            }
            cout << endl;
        }
        return tours;
    }
    return vector<vector<Edge*>>();
}

int vrpTable(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const string& S, const Bag& Xi) {
    // The smallest value such that all vertices below Xi have degree 2 and vertices in Xi have degrees defined by S
    // (with some additional constraints, like the depot can only appear as end point of paths, and there are no cycles (exception root bag), ...)

    // If we know the value already, just look it up.
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
    (*rHashlists[Xi.vid])[S] = vrpRecurse(graph, rHashlists, Xi, edges, 0, 0, degrees, childDegrees, endpoints, childEndpoints);
    return rHashlists[Xi.vid]->at(S);
}

vector<vector<Edge*>> vrpReconstruct(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const string& S, const Bag& Xi) {
    // Reconstruct the vrp tours (get a list of all tours with their tour edges)
    vector<Edge*> edges = getBagEdges(Xi);
    vector<int> degrees = toDegrees(S);
    vector<int> endpoints = toEndpoints(S);
    vector<vector<int>> childEndpoints = vector<vector<int>>(Xi.edges.size());
    vector<vector<int>> childDegrees = vector<vector<int>>(Xi.edges.size());
    for (unsigned int i=0; i<Xi.edges.size(); ++i) {
        childEndpoints[i] = vector<int>();
        childDegrees[i] = vector<int>(degrees.size(), 0);
    }
    return vrpRecurseVector(graph, rHashlists, Xi, edges, 0, 0, degrees, childDegrees, endpoints, childEndpoints);
}

int vrpChildEvaluation(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge*>& edges, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<int>& rEndpoints, vector<vector<int>>& rChildEndpoints, vector<Edge*>* pResultingEdgeList /*=nullptr*/) {
    // This method is the base case for the calculate vrp recurse method - it is the same as tspChildEval, except that it calls the vrpEdgeSelect and vrpTable [TODO?].
    // If we analyzed the degrees of all vertices (i.e. we have a complete combination), return the sum of B values of all children.
    // TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
    // TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
    // TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
    // TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
    // TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
    // This method is exactly the same as tspChildEvaluation, except that it calls the vrpEdgeSelect AND vrpTable
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
    vector<pair<int, vector<int>>> edgeSelectEps = vrpEdgeSelect(0, numeric_limits<int>::max(), 0, graph, Xi, edges, rTargetDegrees, rEndpoints, allChildEndpoints, pResultingEdgeList);
    int val = edgeSelectEps[0].first;
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
                int tableVal = vrpTable(graph, rHashlists, S, Xkid);
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

vector<Edge*>* vrpLookback(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge>& edges, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<int>& rEndpoints, vector<vector<int>>& rChildEndpoints) {
    // TODO
    return nullptr;
}

int vrpRecurse(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge*>& edges, unsigned int i, unsigned int j, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<int>& rEndpoints, vector<vector<int>>& rChildEndpoints) {
    // Select all possible mixes of degrees for all vertices and evaluate them
    //   i = the vertex we currently analyze, j = the child we currently analyze
    //   rTargetDegrees goes from full to empty, rChildDegrees from empty to full, endpoints are the endpoints for each child path
    bool debug = true;
    if (debug) {
        // tree-of-childDegrees          (Xi: i, j)   targetDegrees|endpoints
        cout << dbg("  ", i) << dbg(rChildDegrees) << dbg("  ", Xi.vertices.size() + 9 - i);
        cout << "(X" << Xi.vid << ": " << i << ", " << j << ")   " << dbg(rTargetDegrees) << "|" << dbg(rEndpoints) << endl;
    }

    // Final base case.
    if (i >= Xi.vertices.size())
        return vrpChildEvaluation(graph, rHashlists, Xi, edges, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints);
    // Base case: if we can't or didn't want to 'spend' this degree, move on
    if (rTargetDegrees[i] == 0 || j >= Xi.edges.size())
        return vrpRecurse(graph, rHashlists, Xi, edges, i + 1, 0, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints);
    const Bag& Xj = *dynamic_cast<Bag*>(Xi.edges[j]->Other(Xi));
    // Base case: if the current bag (must be child) does not contain the vertex to analyze, try the next (child) bag
    if (Xi.getParent() == Xi.edges[j]->Other(Xi) || !Xj.ContainsVertex(Xi.vertices[i]))
        return vrpRecurse(graph, rHashlists, Xi, edges, i, j + 1, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints);

    // If the current degree is 2, try letting the child manage it (it = the i-th vertex in Xi)
    int result = numeric_limits<int>::max();
    if (rTargetDegrees[i] == 2 and rChildDegrees[j][i] == 0) {
        vector<int> td = duplicate(rTargetDegrees);
        vector<vector<int>> cds = duplicate(rChildDegrees);
        td[i] = 0;
        cds[j][i] = 2;
        result = vrpRecurse(graph, rHashlists, Xi, edges, i + 1, 0, td, cds, rEndpoints, rChildEndpoints);
    }
    // If the current degree is at least 1 (which it is if we get here), try to combine it (for all other vertices) in a hamiltonian path
    // (all these 'hamiltonian paths' are added as single edges, they are merged later, in cycle check (via ???))
    for (unsigned int k=i+1; k<Xi.vertices.size(); ++k) {
        // Stay in {0, 1, 2} (and make sure child k has the i-th vertex of Xi as well)
        if (rTargetDegrees[k] < 1 || rChildDegrees[j][k] > 1 || !Xj.ContainsVertex(Xi.vertices[k]))
            continue;
        // Don't add edges twice
        if (inEndpoints(rChildEndpoints[j], Xi.vertices[i]->vid, Xi.vertices[k]->vid, 3))
            continue;
        // Check if the added vertex (vertices) doesn't (don't) add too much to the capacity
        int demandForI = 0;
        int demandForK = 0; // TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO

        // Init parameter arrays
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
                eps[j].erase(eps[j].begin() + posBeginK, eps[j].begin() + posBeginK + 3);
            }
        }
        else {
            // So now both are new in the list, so we just add (insert) them
            eps[j].push_back(Xi.vertices[i]->vid);
            eps[j].push_back(Xi.vertices[k]->vid);
            eps[j].push_back(-1);
        }
        // TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
        // TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
        //      demands
        // TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
        // TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO

        // We may have to try to analyze the same vertex again if it's degree is higher than 1
        result = min(result, vrpRecurse(graph, rHashlists, Xi, edges, i, j, td, cds, rEndpoints, eps));
    }
    // Also, try not assigning this degree to anyone, we (maybe) can solve it inside Xi
    result = min(result, vrpRecurse(graph, rHashlists, Xi, edges, i, j + 1, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints));
    return result;
}

vector<vector<Edge*>> vrpRecurseVector(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge*>& edges, unsigned int i, unsigned int j, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<int>& rEndpoints, vector<vector<int>>& rChildEndpoints) {
    // TODO
    return vector<vector<Edge*>>();
}

vector<pair<int, vector<int>>> vrpEdgeSelect(int cost, int minimum, unsigned int index, const Graph& graph, const Bag& Xi, const vector<Edge*>& edges, const vector<int>& degrees, vector<int>& rEndpoints, vector<int>& rAllChildEndpoints, vector<Edge*>* pEdgeList /*=nullptr*/) {
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
        if (!cycleCheck(graph, rEndpoints, pEdgeList, rAllChildEndpoints, 3)) {
            if (debug)
                cout << "Edge select (" << index << "): edges contain a cycle" << endl;
            return vector<pair<int, vector<int>>>(); // TODO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        }
        if (debug)
            cout << "Edge select (" << index << "): satisfied: no need to add any more edges, min value: 0" << endl;
        return vector<pair<int, vector<int>>>(); // TODO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    }

    // Base case 2: we have not succeeded yet, but there are no more edges to add, so we failed
    if (index >= edges.size()) {
        if (debug)
            cout << "Edge select (" << index << "): no more edges to add" << endl;
        return vector<pair<int, vector<int>>>(); // TODO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
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
                return vector<pair<int, vector<int>>>(); // TODO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
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
    vector<Edge*> tempEL1;
    vector<Edge*> tempEL2;
    if (pEdgeList != nullptr) {
        tempEL1 = duplicate(*pEdgeList);    // Notice the pointers to stack objects.                                           - OPTIMIZATION: use one uint instead of a vector of 32 edges
        tempEL2 = duplicate(tempEL1);       // They will go out of scope at the end of the function, but that's fine,
    }
    vector<Edge*>* pTempEL1 = &tempEL1;     // we no longer need them after the edges are pushed back to pEdgeList.
    vector<Edge*>* pTempEL2 = &tempEL2;     //
    pTempEL1->push_back(pEdge);
    int val = 0;vrpEdgeSelect(cost + pEdge->Cost, minimum - pEdge->Cost, index + 1, graph, Xi, edges, deg, rEndpoints, rAllChildEndpoints, pTempEL1);
    if (val < numeric_limits<int>::max())
        minimum = min(minimum, pEdge->Cost + val);
    val = 0;vrpEdgeSelect(cost, minimum, index + 1, graph, Xi, edges, degrees, rEndpoints, rAllChildEndpoints, pTempEL2);
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
    if (debug)
        cout << "Edge select (" << index << "): min value: " << minimum << ", edges: " << dbg(pEdgeList) << endl;
    return vector<pair<int, vector<int>>>(); // TODO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
}

//
// Some helper functions
//
bool isDepot(Vertex* pV) {
    // return whether or not this vertex is the depot
    return false; // TODO
}

vector<int> pathDemands(const Graph& graph, const Bag& Xi, const vector<Edge*>& edgeList, const vector<int>& endpoints, const vector<int>& allChildEndpoints) {
    // Loop through the chosen edges and save the demand per path (and the corresponding endpoints: int_ep, int_ep, int_d)
    bool debug = false;

    // Inits
    vector<int> result = vector<int>(endpoints.size(), 0);
    int progressCounter = -3;
    unsigned int edgeCounter = 0;
    unsigned int endpsCounter = 0;
    Vertex* pV = nullptr;

    // Empty case
    if (endpoints.size() == 0)
        return result;

    // Normal case
    while (true) {
        // Dump the state
        if (debug) {
            cout << "path demands dump:" << endl;
            cout << "  endpoints: " << dbg(endpoints) << endl;
            cout << "  edgeList: " << edgeCounter << " - " << dbg(edgeList) << endl;
            cout << "  kid endpoints: " << endpsCounter << " - " << dbg(allChildEndpoints) << endl;
            cout << "  progress: " << progressCounter << " - v: " << (pV==nullptr ? -1 : pV->vid ) << endl << endl;
        }

        // If we completed the path
        if (pV == nullptr || pV->vid == endpoints[progressCounter + 1]) {
            if (pV != nullptr)
                result[progressCounter + 1] = pV->vid; // Store the old vid (to close the last of the three ints - with the second one :P)
            progressCounter += 3;
            if (static_cast<unsigned int>(progressCounter) >= endpoints.size()) {
                if (edgeCounter == edgeList.size() && endpsCounter == allChildEndpoints.size()) {
                    return result;
                } else {
                    cout << "ASSERTKION ERROR (pathDemands): all endpoints are satisfied, but there are edges or endpoints left" << endl;
                    return vector<int>();
                }
            }
            pV = graph.vertices[endpoints[progressCounter]].get();
            result[progressCounter] = pV->vid; // Store the new vid already (open the first of the three ints - with the first one :])

            if (debug) {
                cout << "Completed the path; progress: " << progressCounter << " - v: " << (pV==nullptr ? -1 : pV->vid ) << endl << endl;
            }
        }

        // Find the next vertex (since we just have had the cycleCheck function doing its magic, we don't need to loop here anymore - RIGHT????
        if (allChildEndpoints[endpsCounter] == pV->vid || allChildEndpoints[endpsCounter + 1] == pV->vid) {
            // Update the move variables
            pV = graph.vertices[allChildEndpoints[pV->vid == allChildEndpoints[endpsCounter] ? endpsCounter + 1 : endpsCounter]].get();
            endpsCounter += 3;
        }
        else if (edgeList[edgeCounter]->IsIncidentTo(pV)) {
            // For the first edge only (so that all cities are counted once), add demands for both vertices (cities), so in this line the old one (starting city)
            if (result[progressCounter + 2] == 0)
                result[progressCounter + 2] = pV->demand;

            // Update the move variables
            pV = edgeList[edgeCounter]->Other(*pV);
            ++edgeCounter;

            // Add the demand for the new vertex (go-to city)
            result[progressCounter + 2] += pV->demand;
        }
        else {
            cout << "eps: " << dbg(endpoints) << ", edgelist: " << dbg(edgeList) << ", all kid eps: " << dbg(allChildEndpoints) << endl;
            cout << "ASSERTION ERROR (pathDemands): no more endpoints or edges found according to specs" << endl;
            return vector<int>();
        }
    }
    cout << "ASSERTION ERROR (pathDemand): The code should not come here." << endl;
    return vector<int>();
}
