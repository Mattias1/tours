#include "dp_vrp.h"

#include "graph.h"
#include "treedecomposition.h"
#include "utils.h"
#include <algorithm>
#include <iostream>
#include <limits>
#include <assert.h>
using namespace std;


//
//  The core functions used to calculate the tour for the VRP problem.
//
vector<vector<Edge*>> vrpDP(const TreeDecomposition& TD, bool consoleOutput /*=true*/) {
    // Compute the smallest tour using DP on a tree decomposition.
    bool debug = false;

    // Make sure there is a proper graph and decomposition
    const Bag* pXroot = TD.getRoot();
    int TRUCKS = TD.getOriginalGraph()->trucks;
    int CAPACITY = TD.getOriginalGraph()->capacity;
    if (TD.vertices.size() < 1 || pXroot == nullptr || TD.getOriginalGraph()->vertices.size() < 1)
        assert(false && "ERROR (VRP-DP): Probably no root bag present");
    if (TRUCKS < 1 || CAPACITY < 1)
        assert(false && "ERROR (VRP-DP): Invalid problem parameters");

    // Start the calculation
    unordered_map<string, int> hashlist;
    vector<int> degrees = vector<int>(pXroot->vertices.size(), 2);
    degrees[0] = TRUCKS * 2;
    for (int i=0; i<TD.getOriginalGraph()->vertices.size(); ++i) {
        Vertex* pV = TD.getOriginalGraph()->vertices[i].get();
        if (pV->edges.size() < 2)
            degrees[0] -= 2;
        assert(degrees[0] >= 0 && "ERROR (VRP-DP): too many single city tour exceptions (more than there are trucks)");
    }
    vector<MatchingEdge> endpointsMemoryManager = vector<MatchingEdge>(degrees[0] / 2);
    for (int i=0; i<endpointsMemoryManager.size(); ++i) {
        endpointsMemoryManager[i] = MatchingEdge(0, 0, CAPACITY);
    }
    string S = toTableEntry(*pXroot, degrees, pointerize(endpointsMemoryManager));
    int value = vrpTable(*TD.getOriginalGraph(), hashlist, S, *pXroot);
    if (consoleOutput)
        cout << "VRP cost: " << value << endl;
    if (debug) {
        for (const auto& iter : hashlist)
            cout << "  " << iter.first << ": " << iter.second << endl;
    }

    // Reconstruct the tour
    if (false && value < numeric_limits<int>::max()) { // REMOVE THE FALSE HERE TO MAKE IT CALCULATE THE FOUND TOUR (well, you'd also need to implement the vrp recurse for edges)
        vector<vector<Edge*>> tours = vrpReconstruct(*TD.getOriginalGraph(), hashlist, S, *pXroot);
        for (int i=0; i<tours.size(); ++i)
            tours[i] = removeDoubles(tours[i], TD.getOriginalGraph()->vertices.size());
        if (debug) {
            cout << "\nDP-TSP:\n  Length: " << value << endl;
            for (int i=0; i<tours.size(); ++i) {
                cout << "  Tour #" << i << ": ";
                for (int j=0; j<tours[i].size(); ++j)
                    cout << *tours[i][j] << ", ";
            }
            cout << endl;
        }
        return tours;
    }
    return vector<vector<Edge*>>();
}

int vrpTable(const Graph& graph, unordered_map<string, int>& rHashlist, const string& S, const Bag& Xi) {
    // The smallest value such that all vertices below Xi have degree 2 and vertices in Xi have degrees defined by S
    // (with some additional constraints, like the depot can only appear as end point of paths, and there are no cycles (exception root bag), ...)

    // If we know the value already, just look it up.
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
    int singleCityTourCost = singleCityTourSpecialCaseManager(graph, Xi, degrees);
    // Recursively find all possible combinations that satisfy the parameter arrays
    int value = vrpRecurse(graph, rHashlist, Xi, Yi, 0, 0, degrees, childDegrees, endpoints, childEndpoints);
    if (value < numeric_limits<int>::max()) {
        if (singleCityTourCost < numeric_limits<int>::max())
            value += singleCityTourCost;
        else
            value = numeric_limits<int>::max();
    }
    rHashlist[S] = value;
    return value;
}

vector<vector<Edge*>> vrpReconstruct(const Graph& graph, unordered_map<string, int>& rHashlist, const string& S, const Bag& Xi) {
    // Reconstruct the vrp tours (get a list of all tours with their tour edges)
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
    return vrpRecurseVector(graph, rHashlist, Xi, Yi, 0, 0, degrees, childDegrees, endpoints, childEndpoints);
}

int vrpRecurse(const Graph& graph, unordered_map<string, int>& rHashlist, const Bag& Xi, const vector<Edge*>& Yi, int i, int j, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<MatchingEdge*>& rEndpoints, vector<vector<MatchingEdge*>>& rChildEndpoints, bool ignoreD2D /*=false*/) {
    // Select all possible mixes of degrees for all vertices and evaluate them
    //   i = the vertex we currently analyze, j = the child we currently analyze
    //   rTargetDegrees goes from full to empty, rChildDegrees from empty to full, endpoints are the endpoints for each child path
    bool debug = false; //i==Xi.vertices.size() && Xi.vid==0; // bool debug = true;
    if (debug) {
        // tree-of-childDegrees          (Xi: i, j)   targetDegrees|endpoints
        cout << dbg("  ", i) << dbg(rChildDegrees) << dbg("  ", Xi.vertices.size() + 9 - i);
        cout << "(X" << Xi.vid << ": " << i << ", " << j << ")   " << dbg(rTargetDegrees) << "|" << dbg(rEndpoints) << endl;
        // cout << dbg("  ", i) << "Child endpoints: " << dbg(rChildEndpoints) << endl;
    }

    // Final base case.
    if (i >= Xi.vertices.size())
        return vrpChildEvaluation(graph, rHashlist, Xi, Yi, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints);
    // Base case: if we can't or didn't want to 'spend' this degree, move on
    if (rTargetDegrees[i] == 0 || j >= Xi.edges.size())
        return vrpRecurse(graph, rHashlist, Xi, Yi, i + 1, 0, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints);
    const Bag& Xj = *dynamic_cast<Bag*>(Xi.edges[j]->Other(Xi));
    // Base case: if the current bag (must be child) does not contain the vertex to analyze, try the next (child) bag
    if (Xi.getParent() == Xi.edges[j]->Other(Xi) || !Xj.ContainsVertex(Xi.vertices[i]))
        return vrpRecurse(graph, rHashlist, Xi, Yi, i, j + 1, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints);

    int result = numeric_limits<int>::max();
    // Special case for the depot vertex: try to pass it on (as a matching {0, 0: -1})
    vector<unique_ptr<MatchingEdge>> matchingsMemoryManager;
    if (isDepot(i)) {
        if (!ignoreD2D && rTargetDegrees[i] >= 2) {
            vector<int> targetDegrees = duplicate(rTargetDegrees);
            vector<vector<int>> childDegrees = duplicate(rChildDegrees);
            vector<vector<MatchingEdge*>> childEndpoints = duplicate(rChildEndpoints);
            for (int k=0; k<rEndpoints.size(); ++k) { // This is both the cause of an infinite recursion, and a nescessity to have - solved by the ignoreD2D parameter.
                if (isDepot(rEndpoints[k]->A) && isDepot(rEndpoints[k]->B)) {
                    if (debug)
                        cout << "Add {0, 0: ?}" << endl;
                    targetDegrees[i] -= 2;
                    childDegrees[j][i] += 2;
                    matchingsMemoryManager.push_back(unique_ptr<MatchingEdge>(new MatchingEdge(Xi.vertices[i]->vid, Xi.vertices[i]->vid, rEndpoints[k]->Demand)));
                    childEndpoints[j].push_back(matchingsMemoryManager.back().get()); // No need (nor allowed) to merge matching edges here (a depot is only allowed as endpoint of a path)

                    vector<int> td = duplicate(targetDegrees);
                    vector<vector<int>> cds = duplicate(childDegrees);
                    vector<vector<MatchingEdge*>> eps = duplicate(childEndpoints);
                    result = min(result, vrpRecurse(graph, rHashlist, Xi, Yi, i, j, td, cds, rEndpoints, eps, true));
                    if (debug)
                        cout << "Added {0, 0: ?}" << endl;
                    if (targetDegrees[i] < 2)
                        break;
                }
            }
        }
    }
    // If the current degree is 2, try letting the child manage it (it = the i-th vertex in Xi) (not allowed for depot)
    else if (rTargetDegrees[i] == 2 and rChildDegrees[j][i] == 0) {
        vector<int> td = duplicate(rTargetDegrees);
        vector<vector<int>> cds = duplicate(rChildDegrees);
        td[i] = 0;
        cds[j][i] = 2;
        result = vrpRecurse(graph, rHashlist, Xi, Yi, i + 1, 0, td, cds, rEndpoints, rChildEndpoints);
    }
    // If the current degree is at least 1 (which it is if we get here), try to combine it (for all other vertices) in a hamiltonian path
    // (all these 'hamiltonian paths' are added as single edges, they are merged later, in cycle check (via ???))
    for (int k=i+1; k<Xi.vertices.size(); ++k) {
        // Stay in {0, 1, 2} (and make sure child k has the i-th vertex of Xi as well)
        if (rTargetDegrees[k] < 1 || rChildDegrees[j][k] > 1 || !Xj.ContainsVertex(Xi.vertices[k]))
            continue;
        // Don't add edges twice
        if (inEndpoints(rChildEndpoints[j], Xi.vertices[i]->vid, Xi.vertices[k]->vid))
            continue;

        // Init parameter arrays
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
            if (!MatchingEdge::MergeInto(Xi.vertices[i]->vid, Xi.vertices[k]->vid, eps[j], matchingsMemoryManager, true))
                edgeMightExistAlready = false;
        }
        if (!edgeMightExistAlready) {
            // So now both are new in the list, so we just add (insert) them
            matchingsMemoryManager.push_back(unique_ptr<MatchingEdge>(new MatchingEdge(Xi.vertices[i]->vid, Xi.vertices[k]->vid)));
            eps[j].push_back(matchingsMemoryManager.back().get());
        }
        result = min(result, vrpRecurse(graph, rHashlist, Xi, Yi, i, isDepot(i) ? j:j+1, td, cds, rEndpoints, eps, true));
    }
    // Also, try not assigning this degree right now, it might be solved for for another child, or we (maybe) can solve it inside Xi
    result = min(result, vrpRecurse(graph, rHashlist, Xi, Yi, i, j + 1, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints));
    return result;
}

vector<vector<Edge*>> vrpRecurseVector(const Graph& graph, unordered_map<string, int>& rHashlist, const Bag& Xi, const vector<Edge*>& Yi, int i, int j, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<MatchingEdge*>& rEndpoints, vector<vector<MatchingEdge*>>& rChildEndpoints) {
    // Not impllemented as I don't actually use it anyway.
    // It's disabled for the TSP as well as it's just very slow. Not worth the time to optimize.
    return vector<vector<Edge*>>();
}

int vrpChildEvaluation(const Graph& graph, unordered_map<string, int>& rHashlist, const Bag& Xi, const vector<Edge*>& Yi, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<MatchingEdge*>& rEndpoints, vector<vector<MatchingEdge*>>& rChildEndpoints) {
    // This method is the base case for the calculate vrp recurse method - it is the same as tspChildEval, except that it calls the vrpEdgeSelect and vrpTable [TODO?].
    // If we analyzed the degrees of all vertices (i.e. we have a complete combination), return the sum of B values of all children.
    // This method is exactly the same as tspChildEvaluation, except that it calls the vrpEdgeSelect AND vrpTable
    bool debug = false;
    if (debug) {
        cout << dbg("  ", Xi.vertices.size()) << "Child endpoints (child eval): " << dbg(rChildEndpoints) << endl;
    }

    // Check: all bags (except the root) are not allowed to be a cycle.
    if (rEndpoints.size() == 0 and Xi.getParent() != nullptr) {
        if (debug) {
            cout << "All bags (except root) should have at least one path - no endpoints given" << endl;
        }
        return numeric_limits<int>::max();
    }
    for (int j=0; j<rChildEndpoints.size(); ++j) {
        if (rChildEndpoints[j].size() == 0 && Xi.getParent() != Xi.edges[j]->Other(Xi)) {
            if (debug)
                cout << dbg("  ", Xi.vertices.size()) << "All child bags should have at least one path." << endl;
            return numeric_limits<int>::max();
        }
    }

    // Base cost: the edges needed inside this Xi to account for the (target) degrees we didn't pass on to our children.
    vector<MatchingEdge*> allChildEndpoints = flatten(rChildEndpoints);
    vector<pair<int, int>> edgeSelectEps = vrpEdgeSelect(0, numeric_limits<int>::max(), 0, graph, Xi, Yi, rTargetDegrees, rEndpoints, allChildEndpoints);

    if (debug) {
        cout << dbg("  ", Xi.vertices.size()) << "Nr of edge selections: " << edgeSelectEps.size() << endl;
    }

    // Loop all possible edge selctions
    int resultValue = numeric_limits<int>::max();
    for (int t=0; t<edgeSelectEps.size(); ++t) {
        int edgeValue = edgeSelectEps[t].first;
        int edgeListBits = edgeSelectEps[t].second;
        // Leaf case should be threated differently here (assuming more than 1 node in the tree decomposition)
        if (Xi.edges.size() == 1 && Xi.getParent() != nullptr) {
            // Abuse the fact that Yi is sorted, so we don't have to bother about the others and can return the first one
            if (debug) {
                cout << "Leaf bag, Local edge selection cost: " << edgeValue << ", Yi: " << dbg(Yi);
                cout << ", degrees: " << dbg(rTargetDegrees) << ", endpoints: " << dbg(rEndpoints) << endl;
            }
            return edgeValue;
        }

        vector<Edge*> edgeList;
        addToEdgeListFromBits(Yi, &edgeList, edgeListBits);
        vector<vector<vector<MatchingEdge>>> possibleMatchings = allChildMatchings(graph, Xi, Yi, edgeList, rEndpoints, rChildEndpoints);
        if (debug)
            cout << dbg("  ", Xi.vertices.size()) << "possibleMatchings size: " << possibleMatchings.size() << ", endpoints: " << dbg(rEndpoints) << ", childEps: " << dbg(rChildEndpoints) << endl;

        // Loop all possible demands in the matchings
        if (possibleMatchings.size() == 0)
            continue;
        if (debug)
            cout << dbg("  ", Xi.vertices.size()) << "possibleMatchings[0] size: " << possibleMatchings[0].size() << endl;
        for (int i=0; i<possibleMatchings[0].size(); ++i) {
            if (edgeValue < numeric_limits<int>::max()) {
                if (debug) {
                    cout << dbg("  ", Xi.vertices.size()) << "Local edge selection cost: " << edgeValue << ", Yi: " << dbg(Yi);
                    cout << ", degrees: " << dbg(rTargetDegrees) << ", endpoints: " << dbg(rEndpoints) << endl;
                }
                int value = edgeValue;
                for (int j=0; j<rChildDegrees.size(); ++j) {
                    const Bag& Xkid = *dynamic_cast<Bag*>(Xi.edges[j]->Other(Xi));
                    if (Xi.getParent() != &Xkid) {
                        // Strip off the vertices not in Xkid and add degrees 2 for vertices not in Xi
                        vector<int> kidDegrees(Xkid.vertices.size(), 2);
                        for (int p=0; p<Xkid.vertices.size(); ++p) {
                            for (int q=0; q<Xi.vertices.size(); ++q)
                                if (Xkid.vertices[p] == Xi.vertices[q]) {
                                    kidDegrees[p] = rChildDegrees[j][q];
                                    break;
                                }
                        }
                        // Parse these to the DP-table string S
                        string S = toTableEntry(Xkid, kidDegrees, possibleMatchings[j][i]);
                        int tableVal = vrpTable(graph, rHashlist, S, Xkid);
                        if (debug) {
                            cout << dbg("  ", Xi.vertices.size()) << "child " << S << ": " << tableVal << " - " << ", cds: " << dbg(rChildDegrees[j]);
                            cout << ", degrees: " << dbg(kidDegrees) << ", endpoints: " << dbg(rChildEndpoints[j]) << ", Ykid: " << dbg(Xkid.GetBagEdges()) << endl;
                        }
                        // Add to that base cost the cost of paths nescessary to satisfy the degrees.
                        if (tableVal == numeric_limits<int>::max()) {
                            if (debug)
                                cout << dbg("  ", Xi.vertices.size()) << "value for this child is int::max" << endl;
                            value = numeric_limits<int>::max();
                            break;
                        }
                        value += tableVal;
                    }
                }
                if (debug) {
                    cout << dbg("  ", Xi.vertices.size()) << "Min cost for X" << Xi.vid << " with these child-degrees: " << value << endl;
                }
                if (value < resultValue) {
                    resultValue = value;
                }
            }
            else if (debug) {
                cout << dbg("  ", Xi.vertices.size()) << "No local edge selection found" << endl;
            }
        }
    }
    if (debug)
        cout << dbg("  ", Xi.vertices.size()) << "Final result: " << resultValue << endl;
    return resultValue;
}

vector<Edge*>* vrpLookback(const Graph& graph, unordered_map<string, int>& rHashlist, const Bag& Xi, const vector<Edge*>& Yi, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<int>& rEndpoints, vector<vector<int>>& rChildEndpoints) {
    // Not impllemented as I don't actually use it anyway.
    // It's disabled for the TSP as well as it's just very slow. Not worth the time to optimize.
    return nullptr;
}

vector<pair<int, int>> vrpEdgeSelect(int cost, int minimum, int index, const Graph& graph, const Bag& Xi, const vector<Edge*>& Yi, const vector<int>& degrees, vector<MatchingEdge*>& rEndpoints, vector<MatchingEdge*>& rAllChildEndpoints, int edgeListBits /*=0*/) {
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
        addToEdgeListFromBits(Yi, &edgeList, edgeListBits);
        if (!vrpCycleCheck(graph, rEndpoints, &edgeList, rAllChildEndpoints)) {
            if (debug)
                cout << "Edge select (" << index << "): edges contain a cycle (or wrong demand)" << endl;
            return vector<pair<int, int>>();
        }
        if (debug)
            cout << "Edge select (" << index << "): satisfied. Cost: " << cost << ", edgeListBits: " << edgeListBits << endl;
        return { make_pair(cost, edgeListBits) };
    }

    // Base case 2: we have not succeeded yet, but there are no more edges to add, so we failed
    if (index >= Yi.size()) {
        if (debug)
            cout << "Edge select (" << index << "): no more edges to add" << endl;
        return vector<pair<int, int>>();
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
                return vector<pair<int, int>>();
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
    vector<pair<int, int>> result = vrpEdgeSelect(cost + pEdge->Cost, minimum - pEdge->Cost, index + 1, graph, Xi, Yi, deg, rEndpoints, rAllChildEndpoints, edgeListBits | 1 << index);
    pushBackList(&result, vrpEdgeSelect(cost, minimum, index + 1, graph, Xi, Yi, degrees, rEndpoints, rAllChildEndpoints, edgeListBits));
    return result;
}

vector<vector<vector<MatchingEdge>>> allChildMatchings(const Graph& graph, const Bag& Xi, const vector<Edge*>& Yi, const vector<Edge*>& edgeList, const vector<MatchingEdge*>& endpoints, const vector<vector<MatchingEdge*>>& childEndpoints) {
    // Find all permutations of demands (capacities) for the children
    // That's going to be quite a number of different permutations to try I think - will need some form of optimization (pruning) here
    // Possible: start with high demand, see what the actual used demand is, and then see if it fits (is this even possible? I doubt it now)

    // Returns: vector, size = nr of child bags (+1 for parent bag, which is left empty as usual)
    //            vector, size = nr of combinations
    //              matchings-list, size = nr of matchings (or paths) for this child

    // The idea:
    // - First list all matchings and the child they belong to per path:
    //   Example: Path 0: Has matching 0 and 2 from child 0, and matching 2 from child 2
    //            So pathList[0] = ((0,0), (0,2), (2,2);
    // - Then get all the combinations and store it in the vectors (recursive???)
    bool debug = false;

    if (debug) {
        cout << endl << "X" << Xi.vid << ", edgeList: " << dbg(edgeList) << ", endpoints: " << dbg(endpoints) << ", childEndpoints: " << dbg(childEndpoints) << endl << endl;
    }

    // Empty case
    assert(endpoints.size() > 0);

    // Inits
    vector<vector<pair<int, int>>> pathList = vector<vector<pair<int, int>>>(endpoints.size());
    vector<int> pathDemands = vector<int>(endpoints.size()); // The demands left to distribute for the main paths (after the edge selection).
    int progressCounter = -1;
    vector<vector<bool>> freeEndpoints;
    for (int j=0; j<childEndpoints.size(); ++j)
        freeEndpoints.push_back(vector<bool>(childEndpoints[j].size(), true));
    vector<bool> freeEdges = vector<bool>(edgeList.size(), true);
    Vertex* pV = nullptr;
    int targetVid = -1;
    // pathList:
    //     vector, size = nr of paths
    //       vector, size = nr of subpaths for this path
    //         pair: (child-index, matching-index (in the child's matching list))

    //
    // Step 1: find all sub paths (paths in child bags) of all main paths (paths for the current bag)
    //
    while (true) {
        // Dump the state
        if (debug) {
            cout << "all child ep-possibilities dump:" << endl;
            cout << "  endpoints: " << dbg(endpoints) << ", freeEndpoints: " << dbg(freeEndpoints) << ", freeEdges: " << dbg(freeEdges) << endl;
            cout << "  targetVid: " << targetVid << ", progress: " << progressCounter << ", v: " << (pV==nullptr ? -1 : pV->vid ) << ", pathList: " << dbg(pathList) << endl << endl;
        }

        // If we completed a path
        if (pV == nullptr || pV->vid == targetVid) {
            // Update some loop variables
            ++progressCounter;
            if (progressCounter >= endpoints.size()) {
                //
                // Step 2: Find the actual permutations
                //
                if (debug) {
                    cout << "Completed the path.\n  pathDemands: " << dbg(pathDemands) << ", pathList: " << dbg(pathList) << endl;
                }

                // Check if there is not obviously too little capacity for the sub-paths
                for (int i=0; i<pathDemands.size(); ++i) {
                    if (pathDemands[i] < 2 * pathList[i].size()) { // TODO: what's that 2 doing here?
                        // TODO: Ehhhh, yeah, what now? How to return an error (int::max) here?
                        if (debug)
                            cout << "Too little capacity for the sub path " << i << endl;
                        return vector<vector<vector<MatchingEdge>>>();
                    }
                }

                // Get the numbers for the sub-path demands
                vector<vector<vector<int>>> allSubPathDemands = vector<vector<vector<int>>>(pathDemands.size());
                for (int i=0; i<pathDemands.size(); ++i) {
                    if (pathList[i].size() > 0) {
                        vector<int> loop = vector<int>(pathList[i].size(), 0);
                        distributeDemands(allSubPathDemands[i], loop, pathDemands[i], pathList[i].size());
                    }
                }
                if (debug) {
                    cout << "  allSubPathDemands: " << endl;
                    for (int i=0; i<allSubPathDemands.size(); ++i)
                        cout << "    " << i << ": " << dbg(allSubPathDemands[i]) << ", size: " << allSubPathDemands[i].size() << endl;
                }

                // Create all possible child endpoint possibilities
                vector<vector<vector<MatchingEdge>>> result = vector<vector<vector<MatchingEdge>>>(childEndpoints.size());
                vector<vector<MatchingEdge>> loop = vector<vector<MatchingEdge>>(childEndpoints.size());
                for (int j=0; j<loop.size(); ++j) {
                    loop[j] = vector<MatchingEdge>(childEndpoints[j].size());
                }
                fillAllChildMatchings(result, loop, 0, childEndpoints, pathList, allSubPathDemands);
                if (debug) {
                    cout << "  result: " << endl;
                    for (int j=0; j<result.size(); ++j)
                        cout << "    " << j << ": " << dbg(result[j]) << ", size: " << result[j].size() << endl;
                    cout << endl;
                }
                return result;
            }
            pV = graph.vertices[endpoints[progressCounter]->A].get();
            if (debug)
                cout << "pV becomes: " << pV->vid << endl;
            targetVid = endpoints[progressCounter]->B;

            // Initially (no edges found yet) we have this much demand to give to our sub-paths (if any)
            pathDemands[progressCounter] = endpoints[progressCounter]->Demand - pV->demand;

            if (debug) {
                cout << "Completed the path; progress: " << progressCounter << " - v: " << (pV==nullptr ? -1 : pV->vid ) << endl << endl;
            }
        }

        // Find the next vertex
        bool noBreak = true;
        for (int j=0; j<childEndpoints.size(); ++j) {
            for (int i=0; i<childEndpoints[j].size(); ++i) {
                if (childEndpoints[j][i]->IsIncidentTo(pV->vid) && freeEndpoints[j][i]) {
                    // Update the demands of the previous path
                    pathDemands[progressCounter] += pV->demand;

                    // Update loop variables
                    pV = graph.vertices[childEndpoints[j][i]->Other(pV->vid)].get();
                    if (debug)
                        cout << "pV becomes: " << pV->vid << endl;
                    freeEndpoints[j][i] = false;

                    // Update the sub paths of the current (main) path
                    pathList[progressCounter].push_back(make_pair(j, i));

                    // Break both for loops
                    noBreak = false;
                    j = childEndpoints.size();
                    break;
                }
            }
        }
        if (noBreak) {
            // noBreak = true; // Uhh, yeah, doh, if noBreak wasn't true, we wouldn't reach this bit of code right?
            for (int i=0; i<edgeList.size(); ++i) {
                if (edgeList[i]->IsIncidentTo(pV) && freeEdges[i]) {
                    // Update loop variables
                    pV = edgeList[i]->Other(*pV);
                    freeEdges[i] = false;

                    // Update the demands of the current path
                    pathDemands[progressCounter] -= pV->demand;

                    noBreak = false;
                    break;
                }
            }
            if (noBreak) {
                if (debug) {
                    cout << "eps: " << dbg(endpoints) << ", edgelist: " << dbg(edgeList) << ", kid eps: " << dbg(childEndpoints) << endl;
                    cout << "ERROR, no more endpoints or edges found according to specs" << endl;
                }
                return vector<vector<vector<MatchingEdge>>>();
            }
        }
    }
    cout << "ASSERTION ERROR (allChildEpsPossibilities): The code should not come here." << endl;
    assert(false);
    return vector<vector<vector<MatchingEdge>>>();
}

void distributeDemands(vector<vector<int>>& rResult, vector<int>& rLoop, int demandLeft, int sizeLeft) {
    // Find all permutations of demands (or capacities, w/e - int's) for a single path and store them in the result array.
    // So for a given demand of 3 and a size of 2, this will add [3,0], [2,1], [1,2] and [0,3] to the rResult list (rLoop initialized as vector of size 2).
    bool debug = false;
    if (debug) {
        // result : [], loop: [0], demandLeft: 7, sizeLeft: 2
        cout << "result: " << dbg(rResult) << ", loop: " << dbg(rLoop) << ", demandLeft: " << demandLeft << ", sizeLeft: " << sizeLeft << endl;
    }

    // Base case
    if (sizeLeft == 1) {
        rLoop[rLoop.size() - 1] = demandLeft;
        rResult.push_back(duplicate(rLoop));
        return;
    }

    // Normal case
    // for (int d = demandLeft - 2*(sizeLeft - 1); d>=2; --d) { // No minimum of 2, depot vertex can be endpoint(s).
    for (int d = demandLeft; d>=0; --d) {
        rLoop[rLoop.size() - sizeLeft] = d;
        distributeDemands(rResult, rLoop, demandLeft - d, sizeLeft - 1);
    }
}

void fillAllChildMatchings(vector<vector<vector<MatchingEdge>>>& rResult, vector<vector<MatchingEdge>>& rLoop, int pathIndex, const vector<vector<MatchingEdge*>>& childEndpoints, const vector<vector<pair<int, int>>>& pathList, const vector<vector<vector<int>>>& allSubPathDemands) {
    // Now we have all the building blocks, fill the allChildMatchings vector (result):
    // So, we have a list of paths, a list of child bags and a list of demands (per path).
    // Paths and bags are kind of independend, so now we have to sort of merge them, but not quite.... (I THINK OK, I DONT KNOW MY OWN FUNCTION ANYMORE).

    // This is huge, no really, huge.
    // Basically: do a recursive for-loop for every sub-path demand list.
    bool debug = false;
    if (debug) {
        cout << "result size: " << rResult.size() << ", loop: " << dbg(rLoop) << ", pathidx: " << pathIndex << ", ceps: " << dbg(childEndpoints);
        cout << ", pathlist: " << dbg(pathList) << ", all subpath demands size: " << allSubPathDemands.size() << endl;
    }

    // The arguments:
    // result:
    //     vector, size = nr of child bags (+1 for parent bag)
    //       vector, size = nr of combinations (I don't know the amount, but it increases every time)
    //         matchings-list, size = nr of matchings (or paths) for this child
    // pathList:
    //     vector, size = nr of paths
    //       vector, size = nr of subpaths for this path
    //         pair: (child-index, matching-index (in the child's matching list))
    // allSubPathDemands:
    //     vector, size = nr of paths
    //       vector, size = nr of possibilities
    //         vector, size = nr of subpaths for this path

    // Base case
    if (pathIndex == pathList.size()) {
        // Add the loop-matchings to the result list (which is initialized only for the #childs)
        for (int j=0; j<rResult.size(); j++) {
            rResult[j].push_back(duplicate(rLoop[j]));
            // Note that it is not nescessary to duplicate the whole matching (it wont be edited, but replaced),
            // but since this is not a pointer that's probably what happends
        }
        return;
    }

    // Normal case
    for (int possibility=0; possibility<allSubPathDemands[pathIndex].size(); ++possibility) {
        // Add the subpath demands of the current path to the loop list
        for (int subPath=0; subPath<pathList[pathIndex].size(); ++subPath) {
            int j = pathList[pathIndex][subPath].first;   // The first int in the pair is the child-index in the bag's edgelist
            int i = pathList[pathIndex][subPath].second;  // The second int in the pair is the matching-index in the child's endpoints list
            int demand = allSubPathDemands[pathIndex][possibility][subPath];
            rLoop[j][i] = MatchingEdge(childEndpoints[j][i]->A, childEndpoints[j][i]->B, demand);
        }
        fillAllChildMatchings(rResult, rLoop, pathIndex + 1, childEndpoints, pathList, allSubPathDemands);
    }

    // In case of a main path that has no subpaths, continue with the next main path
    if (allSubPathDemands[pathIndex].size() == 0) {
        fillAllChildMatchings(rResult, rLoop, pathIndex + 1, childEndpoints, pathList, allSubPathDemands);
    }
}

int singleCityTourSpecialCaseManager(const Graph& graph, const Bag& Xi, vector<int>& rDegrees) {
    // There is a special case for the tours with a single city. These cannot be solved by the DP, because it has as assumption that every city has at least two neighbours.
    // However, this case is trivial to manage up front, so we'll just exclude them from the DP and then add the cost for these tours afterwards.
    // In this method, we exclude them from the degrees list, and we give the cost for just these tours.
    // Note that this method assumes that the starting amount of {0, 0} matching edges is corrected already.
    int totalWeight = 0;
    for (int i=0; i<Xi.vertices.size(); ++i) {
        Vertex* pV = Xi.vertices[i];
        if (pV->edges.size() < 2) {
            assert(pV->edges.size() == 1 && pV->IsConnectedTo(graph.vertices[0].get()) && "ERROR: There is a vertex with degree < 2 and it's not a single city-tour case.");
            rDegrees[i] = 0;
            totalWeight += 2 * pV->edges[0]->Cost;
        }
    }
    return totalWeight;
}

bool vrpCycleCheck(const Graph& graph, const vector<MatchingEdge*>& endpoints, vector<Edge*>* pEdgeList, vector<MatchingEdge*>& rAllChildEndpoints) {
    // This method returns whether or not the given edge list and all child endpoints provide a set of paths
    // satisfying the endpoints and sorts the edge list in place (true if there is NO cycle).
    // TODO: figure out how this method works in the face of a VRP-cased root bag (as it contains loads of cycles, but all of them begin and end in the same vertex: 0).
    bool debug = false;

    // Inits
    int progressCounter = -1;
    int edgeCounter = 0;
    int endpsCounter = 0;
    Vertex* pV = nullptr;
    int targetVid = 0;
    vector<MatchingEdge*> endpointsClone = duplicate(endpoints);
    int pathDemand;

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
            if (pV != nullptr && pathDemand > endpointsClone[progressCounter]->Demand) {
                if (debug)
                    cout << "ERROR: not enough capacity for this path (demand needed: " << pathDemand << ")" << endl;
                return false;
            }
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
            pathDemand = pV->demand; // Store the yet incorrect demand
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
                pathDemand += pV->demand;
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
                    pathDemand += pV->demand;
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

vector<MatchingEdge> pathDemands(const Graph& graph, const Bag& Xi, const vector<Edge*>& edgeList, const vector<MatchingEdge*>& endpoints, const vector<MatchingEdge*>& allChildEndpoints) {
    // THIS FUNCTGION IS NOT USED ANYMORE (merged with cycleCheck)

    // Loop through the chosen edges and save the demand per path (and the corresponding endpoints: int_ep, int_ep, int_d)
    // What we compute here is the total demand of all vertices per path,
    // assuming the childbags only add a single edge (so no new cities, and therefore no additional demand).
    bool debug = false;

    if (debug) {
        cout << "X" << Xi.vid << ", edgeList: " << dbg(edgeList) << ", endpoints: " << dbg(endpoints) << ", childEndpoints: " << dbg(allChildEndpoints) << endl;
    }

    // Empty case for the root bag (if this is for another bag there is a serious error, and it should trigger an assertion error elsewhere).
    if (endpoints.size() == 0)
        return vector<MatchingEdge>();

    // Inits
    vector<MatchingEdge> result = vector<MatchingEdge>(endpoints.size());
    int progressCounter = -1;
    int edgeCounter = 0;
    int endpsCounter = 0;
    Vertex* pV = nullptr;
    int targetVid = 0;

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
        if (pV == nullptr || pV->vid == targetVid) {
            // Check if the demands are not too high
            if (pV != nullptr && result[progressCounter].Demand > endpoints[progressCounter]->Demand) {
                return vector<MatchingEdge>();
            }
            // Check if we finished
            ++progressCounter;
            if (progressCounter >= endpoints.size()) {
                if (edgeCounter == edgeList.size() && endpsCounter == allChildEndpoints.size()) {
                    return result;
                } else {
                    cout << "ASSERTION ERROR (pathDemands): all endpoints are satisfied, but there are edges or endpoints left" << endl;
                    assert(false);
                }
            }
            // Update move variables
            pV = graph.vertices[endpoints[progressCounter]->A].get();
            targetVid = endpoints[progressCounter]->B;
            result[progressCounter] = MatchingEdge(pV->vid, targetVid, pV->demand); // Store the new matching (with the yet incorrect demand)

            if (debug) {
                cout << "Completed the path; progress: " << progressCounter << " - v: " << (pV==nullptr ? -1 : pV->vid ) << endl << endl;
            }
        }

        // Find the next vertex
        //   (since we just have had the cycleCheck function doing its magic, we don't need to loop here any more)
        //   (however, it does go wrong somewhere) - TODO
        if (allChildEndpoints.size() > endpsCounter && allChildEndpoints[endpsCounter]->IsIncidentTo(pV->vid)) {
            // Update the move variables
            pV = graph.vertices[allChildEndpoints[endpsCounter]->Other(pV->vid)].get();
            ++endpsCounter;

            // Update the demand of the path
            result[progressCounter].Demand += pV->demand;
        }
        else if (edgeList[edgeCounter]->IsIncidentTo(pV)) {
            // Update the move variables
            pV = edgeList[edgeCounter]->Other(*pV);
            ++edgeCounter;

            // Add the demand for the new vertex (go-to city)
            result[progressCounter].Demand += pV->demand;
        }
        else {
            cout << "eps: " << dbg(endpoints) << ", edgelist: " << dbg(edgeList) << ", all child eps: " << dbg(allChildEndpoints) << endl;
            cout << "ASSERTION ERROR (pathDemands): no more endpoints or edges found according to specs" << endl;
            assert(false);
        }
    }
    cout << "ASSERTION ERROR (pathDemand): The code should not come here." << endl;
    assert(false);
}

