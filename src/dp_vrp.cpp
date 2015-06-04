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
    for (int i=0; i<hashlists.size(); ++i)
        hashlists[i] = unique_ptr<unordered_map<string, int>>(new unordered_map<string, int>());

    // Start the calculation
    vector<int> degrees = vector<int>(pXroot->vertices.size(), 2);
    string S = fromDegreesEndpoints(degrees, vector<Matching*>());
    int value = vrpTable(*TD.getOriginalGraph(), hashlists, S, *pXroot);
    if (consoleOutput)
        cout << "VRP cost: " << value << endl;
    if (debug) {
        for (int i=0; i<hashlists.size(); ++i) {
            cout << "X" << i << endl;
            for (const auto& iter : *hashlists[i])
                cout << "  " << iter.first << ": " << iter.second << endl;
        }
    }

    // Reconstruct the tour
    if (value < numeric_limits<int>::max()) {
        vector<vector<Edge*>> tours = vrpReconstruct(*TD.getOriginalGraph(), hashlists, S, *pXroot);
        for (int i=0; i<tours.size(); ++i)
            tours[i] = removeDoubles(tours[i], TD.getOriginalGraph()->vertices.size());
        // vector<int> allChildEndpointsParameter;
        // cycleCheck(*TD.getOriginalGraph(), vector<int>(), &tour, allChildEndpointsParameter, 3); // Sort the tour - TODO - ERROR, IT DOESN'T ACTUALLY SORT IT
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

int vrpTable(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const string& S, const Bag& Xi) {
    // The smallest value such that all vertices below Xi have degree 2 and vertices in Xi have degrees defined by S
    // (with some additional constraints, like the depot can only appear as end point of paths, and there are no cycles (exception root bag), ...)

    // If we know the value already, just look it up.
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
    (*rHashlists[Xi.vid])[S] = vrpRecurse(graph, rHashlists, Xi, Yi, 0, 0, degrees, childDegrees, endpoints, childEndpoints);
    return rHashlists[Xi.vid]->at(S);
}

vector<vector<Edge*>> vrpReconstruct(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const string& S, const Bag& Xi) {
    // Reconstruct the vrp tours (get a list of all tours with their tour edges)
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
    return vrpRecurseVector(graph, rHashlists, Xi, Yi, 0, 0, degrees, childDegrees, endpoints, childEndpoints);
}

int vrpChildEvaluation(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge*>& Yi, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<Matching*>& rEndpoints, vector<vector<Matching*>>& rChildEndpoints) {
    // This method is the base case for the calculate vrp recurse method - it is the same as tspChildEval, except that it calls the vrpEdgeSelect and vrpTable [TODO?].
    // If we analyzed the degrees of all vertices (i.e. we have a complete combination), return the sum of B values of all children.
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
    vector<Matching*> allChildEndpoints = flatten(rChildEndpoints);
    vector<pair<int, vector<Matching>>> edgeSelectEps = vrpEdgeSelect(0, numeric_limits<int>::max(), 0, graph, Xi, Yi, rTargetDegrees, rEndpoints, allChildEndpoints);

    // TODO: filter edge selections to remove overrated demands

    // Loop all possible edge selctions
    int resultT = 0;
    int resultVal = numeric_limits<int>::max();
    for (int t=0; t<edgeSelectEps.size(); ++t) {
        int val = edgeSelectEps[t].first;
        vector<Matching> edgeDemands = edgeSelectEps[t].second;
        if (0 <= val && val < numeric_limits<int>::max()) { // TODO: why can val ever be < 0??? Why is this first part of the check here??? It is also there in the python version...
            if (debug) {
                cout << dbg("  ", Xi.vertices.size()) << "Local edge selection cost: " << val << ", Yi: " << dbg(Yi);
                cout << ", degrees: " << dbg(rTargetDegrees) << ", endpoints: " << dbg(rEndpoints) << endl;
            }
            for (int k=0; k<rChildDegrees.size(); ++k) {
                const vector<int>& cds = rChildDegrees[k];
                const vector<Matching*>& ceps = rChildEndpoints[k];
                const Bag& Xkid = *dynamic_cast<Bag*>(Xi.edges[k]->Other(Xi));
                if (Xi.getParent() != &Xkid) {
                    // Strip off the vertices not in Xkid and add degrees 2 for vertices not in Xi
                    vector<int> kidDegrees(Xkid.vertices.size(), 2);
                    for (int p=0; p<Xkid.vertices.size(); ++p) {
                        for (int q=0; q<Xi.vertices.size(); ++q)
                            if (Xkid.vertices[p] == Xi.vertices[q]) {
                                kidDegrees[p] = cds[q];
                                break;
                            }
                    }
                    // Provide the altered endpoints (with edgeDemands!) for as long as they are in the child endpoints
                    vector<Matching*> kidEndpoints(ceps.size());
                    for (int p=0; p<edgeDemands.size(); ++p) {
                        for (int q=0; q<ceps.size(); ++q)
                            if (true) {
                                break;
                                // TODO: REWRITE!!!!!!!!!!!!!!!!!!!!!!!!!!!
                            }
                    }
                    // Parse these to the DP-table string S
                    string S = fromDegreesEndpoints(kidDegrees, kidEndpoints);
                    if (debug) {
                        cout << dbg("  ", Xi.vertices.size()) << "child A: " << val << ", cds: " << dbg(cds);
                        cout << ", degrees: " << dbg(kidDegrees) << ", endpoints: " << dbg(rChildEndpoints[k]) << endl;
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
        if (val < resultVal) {
            resultT = t;
            resultVal = val;
        }
    }
    return resultVal;
}

vector<Edge*>* vrpLookback(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge*>& Yi, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<int>& rEndpoints, vector<vector<int>>& rChildEndpoints) {
    // TODO
    return nullptr;
}

int vrpRecurse(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge*>& Yi, int i, int j, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<Matching*>& rEndpoints, vector<vector<Matching*>>& rChildEndpoints) {
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
        return vrpChildEvaluation(graph, rHashlists, Xi, Yi, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints);
    // Base case: if we can't or didn't want to 'spend' this degree, move on
    if (rTargetDegrees[i] == 0 || j >= Xi.edges.size())
        return vrpRecurse(graph, rHashlists, Xi, Yi, i + 1, 0, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints);
    const Bag& Xj = *dynamic_cast<Bag*>(Xi.edges[j]->Other(Xi));
    // Base case: if the current bag (must be child) does not contain the vertex to analyze, try the next (child) bag
    if (Xi.getParent() == Xi.edges[j]->Other(Xi) || !Xj.ContainsVertex(Xi.vertices[i]))
        return vrpRecurse(graph, rHashlists, Xi, Yi, i, j + 1, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints);

    // If the current degree is 2, try letting the child manage it (it = the i-th vertex in Xi)
    int result = numeric_limits<int>::max();
    if (rTargetDegrees[i] == 2 and rChildDegrees[j][i] == 0) {
        vector<int> td = duplicate(rTargetDegrees);
        vector<vector<int>> cds = duplicate(rChildDegrees);
        td[i] = 0;
        cds[j][i] = 2;
        result = vrpRecurse(graph, rHashlists, Xi, Yi, i + 1, 0, td, cds, rEndpoints, rChildEndpoints);
    }
    // If the current degree is at least 1 (which it is if we get here), try to combine it (for all other vertices) in a hamiltonian path
    // (all these 'hamiltonian paths' are added as single edges, they are merged later, in cycle check (via ???))
    vector<Matching> newMatchingsMemoryManager;
    for (int k=i+1; k<Xi.vertices.size(); ++k) {
        // Stay in {0, 1, 2} (and make sure child k has the i-th vertex of Xi as well)
        if (rTargetDegrees[k] < 1 || rChildDegrees[j][k] > 1 || !Xj.ContainsVertex(Xi.vertices[k]))
            continue;
        // Don't add edges twice
        if (inEndpoints(rChildEndpoints[j], Xi.vertices[i]->vid, Xi.vertices[k]->vid))
            continue;
        // Check if the added vertex (vertices) doesn't (don't) add too much to the capacity
        int demandForI = 0;
        int demandForK = 0; // TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO

        // Init parameter arrays
        vector<int> td = duplicate(rTargetDegrees);
        vector<vector<int>> cds = duplicate(rChildDegrees);
        vector<vector<Matching*>> eps = duplicate(rChildEndpoints);
        // Update the degrees
        td[i] -= 1;
        cds[j][i] += 1;
        td[k] -= 1;
        cds[j][k] += 1;
        // Update the endpoints (update when one (or both) of the endpoints are already in the list, otherwise insert)
        if (cds[j][i] == 2 || cds[j][k] == 2) {
            // So now at least one of the two is not new in the list, so we update it (or them)
            if (!Matching::MergeInto(Xi.vertices[i]->vid, Xi.vertices[k]->vid, eps[j], newMatchingsMemoryManager))
                cout << "ASSERTION ERROR - Matching::MergeInto returns false!!!" << endl;
        }
        else {
            // So now both are new in the list, so we just add (insert) them
            Matching matching = Matching(Xi.vertices[i]->vid, Xi.vertices[k]->vid, -1);
            newMatchingsMemoryManager.push_back(matching);
            eps[j].push_back(&matching);
        }
        // TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
        // TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
        //      demands
        // TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
        // TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO

        // We may have to try to analyze the same vertex again if it's degree is higher than 1
        result = min(result, vrpRecurse(graph, rHashlists, Xi, Yi, i, j, td, cds, rEndpoints, eps));
    }
    // Also, try not assigning this degree to anyone, we (maybe) can solve it inside Xi
    result = min(result, vrpRecurse(graph, rHashlists, Xi, Yi, i, j + 1, rTargetDegrees, rChildDegrees, rEndpoints, rChildEndpoints));
    return result;
}

vector<vector<Edge*>> vrpRecurseVector(const Graph& graph, vector<unique_ptr<unordered_map<string, int>>>& rHashlists, const Bag& Xi, const vector<Edge*>& Yi, int i, int j, vector<int>& rTargetDegrees, vector<vector<int>>& rChildDegrees, vector<Matching*>& rEndpoints, vector<vector<Matching*>>& rChildEndpoints) {
    // TODO
    return vector<vector<Edge*>>();
}

vector<pair<int, vector<Matching>>> vrpEdgeSelect(int cost, int minimum, int index, const Graph& graph, const Bag& Xi, const vector<Edge*>& Yi, const vector<int>& degrees, vector<Matching*>& rEndpoints, vector<Matching*>& rAllChildEndpoints, int edgeListBits /*=0*/) {
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
        if (!cycleCheck(graph, rEndpoints, &edgeList, rAllChildEndpoints)) {
            if (debug)
                cout << "Edge select (" << index << "): edges contain a cycle" << endl;
            vector<Matching> edgeDemands = pathDemands(graph, Xi, edgeList, rEndpoints, rAllChildEndpoints);
            return { make_pair(cost, edgeDemands) };
        }
        if (debug)
            cout << "Edge select (" << index << "): satisfied: no need to add any more edges, min value: 0" << endl;
        return vector<pair<int, vector<Matching>>>();
    }

    // Base case 2: we have not succeeded yet, but there are no more edges to add, so we failed
    if (index >= Yi.size()) {
        if (debug)
            cout << "Edge select (" << index << "): no more edges to add" << endl;
        return vector<pair<int, vector<Matching>>>();
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
                return vector<pair<int, vector<Matching>>>();
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
    vector<pair<int, vector<Matching>>> result = vrpEdgeSelect(cost + pEdge->Cost, minimum - pEdge->Cost, index + 1, graph, Xi, Yi, deg, rEndpoints, rAllChildEndpoints, edgeListBits | 1 << index);
    pushBackList(&result, vrpEdgeSelect(cost, minimum, index + 1, graph, Xi, Yi, degrees, rEndpoints, rAllChildEndpoints, edgeListBits));
    return result;
}

//
// Some helper functions
//
bool isDepot(Vertex* pV) {
    // return whether or not this vertex is the depot
    return false; // TODO
}

vector<Matching> pathDemands(const Graph& graph, const Bag& Xi, const vector<Edge*>& edgeList, const vector<Matching*>& endpoints, const vector<Matching*>& allChildEndpoints) {
    // Loop through the chosen edges and save the demand per path (and the corresponding endpoints: int_ep, int_ep, int_d)
    bool debug = false;

    // Empty case
    if (endpoints.size() == 0)
        return vector<Matching>();

    // Inits
    vector<Matching> result;
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
            ++progressCounter;
            if (progressCounter >= endpoints.size()) {
                if (edgeCounter == edgeList.size() && endpsCounter == allChildEndpoints.size()) {
                    return result;
                } else {
                    cout << "ASSERTION ERROR (pathDemands): all endpoints are satisfied, but there are edges or endpoints left" << endl;
                    return vector<Matching>();
                }
            }
            pV = graph.vertices[endpoints[progressCounter]->A].get();
            targetVid = endpoints[progressCounter]->B;
            result.push_back(Matching(pV->vid, targetVid, 0)); // Store the new matching (with the yet (possibly) uncorrect demand)

            if (debug) {
                cout << "Completed the path; progress: " << progressCounter << " - v: " << (pV==nullptr ? -1 : pV->vid ) << endl << endl;
            }
        }


        //
        // TODO: WRONG!!!!!!!! if there is a child-path first and then an edge, the first vertex of the edge will never be counted.
        //


        // Find the next vertex (since we just have had the cycleCheck function doing its magic, we don't need to loop here anymore - RIGHT????
        if (allChildEndpoints[endpsCounter]->IsIncidentTo(pV->vid)) {
            // Update the move variables
            pV = graph.vertices[allChildEndpoints[endpsCounter]->Other(pV->vid)].get();
            ++endpsCounter;
        }
        else if (edgeList[edgeCounter]->IsIncidentTo(pV)) {
            // For the first edge only (so that all cities are counted once), add demands for both vertices (cities), so in this line the old one (starting city)
            if (result[progressCounter].Demand == 0)
                result[progressCounter].Demand = pV->demand;

            // Update the move variables
            pV = edgeList[edgeCounter]->Other(*pV);
            ++edgeCounter;

            // Add the demand for the new vertex (go-to city)
            result[progressCounter].Demand += pV->demand;
        }
        else {
            cout << "eps: " << dbg(endpoints) << ", edgelist: " << dbg(edgeList) << ", all kid eps: " << dbg(allChildEndpoints) << endl;
            cout << "ASSERTION ERROR (pathDemands): no more endpoints or edges found according to specs" << endl;
            return vector<Matching>();
        }
    }
    cout << "ASSERTION ERROR (pathDemand): The code should not come here." << endl;
    return vector<Matching>();
}

vector<vector<vector<Matching>>> allChildMatchings(const Graph& graph, const Bag& Xi, const vector<Edge*>& Yi, const vector<Edge*>& edgeList, const vector<Matching*>& endpoints, const vector<vector<Matching*>>& childEndpoints) {
    // Find all permutations of demands (capacities) for the children
    // That's going to be quite a number of different permutations to try I think - will need some form of optimization (pruning) here
    // Possible: start with high demand, see what the actual used demand is, and then see if it fits (is this even possible? I doubt it now)

    // Returns: vector, size = nr of child bags (+1 for parent bag)
    //            vector, size = nr of combinations
    //              matchings-list, size = nr of matchings (or paths) for this child

    // The idea:
    // - First list all matchings and the child they belong to per path:
    //   Example: Path 0: Has matching 0 and 2 from child 0, and matching 2 from child 2
    //            So pathList[0] = ((0,0), (0,2), (2,2);
    // - Then get all the combinations and store it in the vectors (recursive???)

    // Assumes cycleCheck method has already been called??? Is this a good assumption??? Probably not... :S
    bool debug = false;

    // Empty case
    if (endpoints.size() == 0) {
        // TODO - I DON'T KNOW WHAT TO DO HERE RIGHT NOW!!!
        // (Probably not return but go on with max capacity value, but also add some initial somethings, so that at least there is a pV to get.)
        return vector<vector<vector<Matching>>>();
    }

    // Inits
    vector<vector<pair<int, int>>> pathList = vector<vector<pair<int, int>>>(endpoints.size());
    vector<int> pathDemands = vector<int>(endpoints.size());
    int progressCounter = -1;
    int lastChild = -1;
    int lastChildEndpoint = -1;
    int lastEdge = -1;
    Vertex* pV = nullptr;
    int targetVid = 0;

    //
    // Step 1: find all sub paths (paths in child bags) of all main paths (paths for the current bag)
    //
    while (true) {
        // Dump the state
        if (debug) {
            cout << "all child ep-possibilities dump:" << endl;
            cout << "  endpoints: " << dbg(endpoints) << endl;
            cout << "  progress: " << progressCounter << " - v: " << (pV==nullptr ? -1 : pV->vid ) << endl << endl;
        }

        // If we completed a path
        if (pV == nullptr || pV->vid == targetVid) {
            // Update some loop variables
            ++progressCounter;
            if (progressCounter >= endpoints.size()) {
                //
                // Step 2: Find the actual permutations
                //
                // Check if there is not obviously too little capacity for the sub-paths
                for (int i=0; i<pathDemands.size(); ++i) {
                    if (pathDemands[i] < 2 * pathList[i].size()) {
                        // TODO: Ehhhh, yeah, what now? How to return an error (int::max) here?
                        return vector<vector<vector<Matching>>>();
                    }
                }

                // Get the numbers for the sub-path demands
                vector<vector<vector<int>>> allSubPathDemands = vector<vector<vector<int>>>(pathDemands.size());
                for (int i=0; i<pathDemands.size(); ++i) {
                    vector<int> loop = vector<int>(pathDemands.size(), 0);
                    distributeDemands(allSubPathDemands[i], loop, pathDemands[i], pathList[i].size());
                }

                // Create all possible child endpoint possibilities
                vector<vector<vector<Matching>>> result = vector<vector<vector<Matching>>>(childEndpoints.size());
                vector<vector<Matching>> loop = vector<vector<Matching>>(childEndpoints.size());
                for (int j=0; j<loop.size(); ++j) {
                    loop[j] = vector<Matching>(childEndpoints[j].size());
                }
                fillAllChildMatchings(result, loop, 0, childEndpoints, pathList, allSubPathDemands);
                return result;
            }
            pV = graph.vertices[endpoints[progressCounter]->A].get();
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
                if (childEndpoints[j][i]->IsIncidentTo(pV->vid) && lastChild != j && lastChildEndpoint != i) { // The depot vertex might make this an infinite loop, pay attention here - TODO
                    // Update the demands of the previous path
                    pathDemands[progressCounter] += pV->demand;

                    // Update loop variables
                    pV = graph.vertices[childEndpoints[j][i]->Other(pV->vid)].get();
                    lastChild = j;
                    lastChildEndpoint = i;

                    // Update the sub paths of the current (main) path
                    pathList[progressCounter].push_back(make_pair(j, i));

                    noBreak = false;
                    break;
                }
            }
        }
        if (noBreak) {
            // noBreak = true; // Uhh, yeah, doh, if noBreak wasn't true, we wouldn't reach this bit of code right?
            for (int i=0; i<edgeList.size(); ++i) {
                if (edgeList[i]->IsIncidentTo(pV) && lastEdge != i) { // The depot vertex might make this an infinite loop, pay attention here - TODO
                    // Update loop variables
                    pV = edgeList[i]->Other(*pV);
                    lastEdge = i;

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
                return vector<vector<vector<Matching>>>();
            }
        }
    }
    cout << "ASSERTION ERROR (allChildEpsPossibilities): The code should not come here." << endl;
    return vector<vector<vector<Matching>>>();
}

void distributeDemands(vector<vector<int>>& rResult, vector<int>& rLoop, int demandLeft, int sizeLeft) {
    // Find all permutations of demands (or capacities, w/e - int's with min value 2) for a single path and store them in the result array.
    // So for a given demand of 6 and a size of 2, this will add [4,2], [3,3] and [2,4] to the rResult list (rLoop initialized as vector of size 2).

    // Base case
    if (sizeLeft == 1) {
        rLoop[rLoop.size() - 1] = demandLeft;
        rResult.push_back(duplicate(rLoop));
        return;
    }

    // Normal case
    for (int d = demandLeft - 2*(sizeLeft - 1); d>=2; ++d) {
        rLoop[rLoop.size() - sizeLeft] = d;
        distributeDemands(rResult, rLoop, demandLeft - d, sizeLeft - 1);
    }
}

void fillAllChildMatchings(vector<vector<vector<Matching>>>& rResult, vector<vector<Matching>>& rLoop, int pathIndex, const vector<vector<Matching*>>& childEndpoints, const vector<vector<pair<int, int>>>& pathList, const vector<vector<vector<int>>>& allSubPathDemands) {
    // Now we have all the building blocks, fill the allChildMatchings vector (result).
    // This is huge, no really, huge.
    // Basically: do a recursive for-loop for every sub-path demand list.

    // The arguments:
    // result:
    //     vector, size = nr of child bags (+1 for parent bag)
    //       vector, size = nr of combinations
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
            rLoop[j][i] = Matching(childEndpoints[j][i]->A, childEndpoints[j][i]->B, allSubPathDemands[pathIndex][possibility][subPath]);
        }
        fillAllChildMatchings(rResult, rLoop, pathIndex + 1, childEndpoints, pathList, allSubPathDemands);
    }
}
