#include "treedecomposition.h"

#include <algorithm>
#include "utils.h"
#include <iostream>
#include <assert.h>
using namespace std;

//
//  Tree decomposition
//
TreeDecomposition::TreeDecomposition(Graph* originalGraph)
    :pRoot(nullptr),
    pOriginalGraph(unique_ptr<Graph>(originalGraph))
{ }

bool TreeDecomposition::ReadFileLine(int& rState, string line) {
    // Handle one line of a file (note that the in place editing of the string in trim is a good thing).
    int startVid = 1; // Apparently ppl like to have their lists 1-based, rather than 0-based.
    vector <string> l = split(trim(line), ' ');
    // Important file parameters
    if (comp(line, "NAME : ")) {
        this->name = l[2];
        return true;
    }
    if (comp(line, "EOF")) {
        rState = 0;
        return false;
    }

    // Vertices and edges
    if (comp(line, "NODE_COORD_SECTION") || comp(line, "EDGE_SECTION")) {
        return false;
    }
    if (comp(line, "BAG_COORD_SECTION")) {
        rState = 3;
        return true;
    }
    if (comp(line, "BAG_EDGE_SECTION")) {
        rState = 4;
        return true;
    }

    // Add vertices, edges, bags or bag edges
    if (rState == 3) {
        if (l.size() <= 3)
            cout << "ERROR: The TD ReadFileLine expects a bag, but it doesn't even gets three strings in the first place." << endl;
        // Create a new bag
        unique_ptr<Bag> pBag = unique_ptr<Bag>(new Bag(stoi(l[0]) - startVid, stoi(l[1]), stoi(l[2])));
        for (int i=3; i<l.size(); ++i) {
            // Add a specific vertex to a bag.
            pBag->vertices.push_back( this->pOriginalGraph->vertices[stoi(l[i]) - startVid].get() );
        }
        // Add the bag to the tree
        this->vertices.push_back(move(pBag));
        return true;
    }
    if (rState == 4) {
        if (l.size() < 2 || l.size() > 3)
            cout << "ERROR: The TD ReadFileLine expects an edge, but it doesn't get two or three strings in the first place." << endl;
        // Add the edge to the edgelist of it's endpoints
        Vertex* pA = this->vertices[stoi(l[0]) - startVid].get();
        Vertex* pB = this->vertices[stoi(l[1]) - startVid].get();
        shared_ptr<Edge> pE = shared_ptr<Edge>(new Edge(pA, pB));
        pA->edges.push_back(pE);
        pB->edges.push_back(pE);
        return true;
    }

    return false;
}

string TreeDecomposition::ToFileString() const {
    // This is supposed to be placed after the normal graph.ToFileString method.
    int startVid = 1;
    if (this->vertices.size() == 0)
        return "";
    string s = "BAG_COORD_SECTION\n";
    for (int i=0; i<this->vertices.size(); ++i) {
        Bag* pBag = dynamic_cast<Bag*>(this->vertices[i].get());
        s += to_string(pBag->vid + startVid) + " " + to_string(pBag->x) + " " + to_string(pBag->y);
        for (int j=0; j<pBag->vertices.size(); ++j) {
            s += " " + to_string(pBag->vertices[j]->vid + startVid);
        }
        s += "\n";
    }
    s += "BAG_EDGE_SECTION\n";
    for (int i=0; i<this->vertices.size(); ++i) {
        Vertex* pBag = this->vertices[i].get();
        for (int j=0; j<pBag->edges.size(); ++j) {
            shared_ptr<Edge> pE = pBag->edges[j];
            if (pBag->vid < pE->Other(*pBag)->vid)
                s += to_string(pE->pA->vid + startVid) + " " + to_string(pE->pB->vid + startVid) + "\n";
        }
    }
    return s;
}

int TreeDecomposition::GetTreeWidth() const {
    // Return the tree width of this tree decomposition
    int result = 0;
    for (int i=0; i<this->vertices.size(); ++i) {
        Bag* pBag = dynamic_cast<Bag*>(this->vertices[i].get());
        if (pBag->vertices.size() > result)
            result = pBag->vertices.size();
    }
    return result - 1;
}

bool TreeDecomposition::CreateRoot(bool adjustCoordinates) {
    if (this->vertices.size() <= 0)
        return false;
    return this->CreateRoot(dynamic_cast<Bag*>(this->vertices[0].get()), adjustCoordinates);
}
bool TreeDecomposition::CreateRoot(Bag* pRoot, bool adjustCoordinates) {
    if (pRoot == nullptr)
        return false;
    pRoot->SetParentsRecursive(nullptr, adjustCoordinates);
    this->pRoot = pRoot;
    return true;
}

void TreeDecomposition::MinimumDegree(bool depotInAllBags /*= false*/) {
    // Check if the treedecompoistion is empty
    if (this->vertices.size() != 0 || this->pRoot != nullptr) {
        cout << "ERROR (Minimum degree): MinimumDegree called on a non empty tree decomposition" << endl;
        return;
    }

    // Create a tree decomposition (with wrong vids)
    this->permutationToTreeDecomposition(depotInAllBags);

    // Root the tree and give the bags some nice coordinates so that it actually draws like a tree with the root on top.
    this->CreateRoot(true);

    // Optimization: if a bag is contained in its parent, we don't really need it
    this->pRoot->TrimBagsRecursive(this);

    // Fix vids
    for (int i=0; i<this->vertices.size(); ++i) {
        // The original algorithm assigns the vids from the vertices of the original graph.
        // We fix them here to ensure our that vertices in a list are indexed by their vids.
        this->vertices[i]->vid = i;
    }
}

void TreeDecomposition::permutationToTreeDecomposition(bool depotInAllBags) {
    // rEdgeList:    This list will be filled with int-pairs that should become the edges in the final tree decomposition
    //               (but the bags might not be created yet) - provide it with an empty list.
    // Note: correct the vid's from each bag later; once the edges in the edge list are added
    unique_ptr<Graph> pGraphCopy = this->pOriginalGraph->DeepCopy();
    vector<bool> freeVertices = vector<bool>(this->pOriginalGraph->vertices.size(), true);
    vector<Vertex*> pendingEdges;

    for (int i=0; i<this->pOriginalGraph->vertices.size(); ++i) {
        Vertex* pV = nullptr;
        int minDegree = numeric_limits<int>::max();
        for (int j=0; j<this->pOriginalGraph->vertices.size(); ++j) {
            if (freeVertices[j]) {
                Vertex* pCurrentCopy = pGraphCopy->vertices[j].get();
                int degree = 0;
                for (int k=0; k<pCurrentCopy->edges.size(); ++k)
                    if (freeVertices[pCurrentCopy->edges[k]->Other(*pCurrentCopy)->vid])
                        ++degree;
                if (degree < minDegree) {
                    pV = this->pOriginalGraph->vertices[j].get();
                    minDegree = degree;
                }
            }
        }
        assert(pV != nullptr && "ERROR (MinDegree): Can't find a vertex with minimum degree");
        freeVertices[pV->vid] = false;
        Vertex* pVCopy = pGraphCopy->vertices[pV->vid].get();
        unique_ptr<Bag> pBagMemoryManager = unique_ptr<Bag>(new Bag(pV->vid, 240, 60 + 120 * i));
        Bag* pBag = pBagMemoryManager.get();

        // Add the (free) neighbourhood of pV to the bag (and, when solving the VRP, make sure we add the depot (as first one))
        if (depotInAllBags && !isDepot(pV))
            pBag->vertices.push_back(this->pOriginalGraph->vertices[0].get());
        pBag->vertices.push_back(pV);
        for (int j=0; j<pVCopy->edges.size(); ++j) {
            Vertex* pNeighbourCopy = pVCopy->edges[j]->Other(*pVCopy);
            if (freeVertices[pNeighbourCopy->vid]) {
                // We found a vertex that is both a 'neighbour' of pV and still in the vertex list - so add it
                if (!depotInAllBags || !isDepot(pNeighbourCopy))
                    pBag->vertices.push_back(this->pOriginalGraph->vertices[pNeighbourCopy->vid].get());
            }
        }

        // Add the bag to the tree decomposition
        this->vertices.push_back(move(pBagMemoryManager));

        // Clique-ify the neighbourhood of this vertex.
        for (int a=0; a<pBag->vertices.size() - 1; ++a) {
            Vertex* pACopy = pGraphCopy->vertices[pBag->vertices[a]->vid].get();
            for (int b=a + 1; b<pBag->vertices.size(); ++b) {
                Vertex* pBCopy = pGraphCopy->vertices[pBag->vertices[b]->vid].get();
                if (!pACopy->IsConnectedTo(pBCopy)) {
                    // Ok, a and b are not connected - add the edge (to the copied graph of course - we're not modding the original graph)
                    shared_ptr<Edge> pECopy = shared_ptr<Edge>(new Edge(pACopy, pBCopy));
                    pACopy->edges.push_back(pECopy);
                    pBCopy->edges.push_back(pECopy);
                }
            }
        }

        // And find the right edge for in the tree decomposition (an edge to the first next neighbour of v in the vertex list)
        for (int j=pendingEdges.size() - 1; j>=0; --j) {
            if (pVCopy->IsConnectedTo(pendingEdges[j])) {
                Vertex* pBagOther = nullptr;
                for (int k=0; k<this->vertices.size(); ++k)
                    if (this->vertices[k]->vid == pendingEdges[j]->vid) {
                        pBagOther = this->vertices[k].get();
                        break;
                    }
                assert(pBagOther != nullptr && "ERROR (MinDegree): Can't find a bag with correct vid");
                shared_ptr<Edge> pE = shared_ptr<Edge>(new Edge(pBag, pBagOther));
                pBag->edges.push_back(pE);
                pBagOther->edges.push_back(pE);
                pendingEdges.erase(pendingEdges.begin() + j);
            }
        }
        pendingEdges.push_back(pVCopy);
    }
}


//
//  Bag
//
Bag::Bag(int vid, int x, int y)
    :Vertex::Vertex(vid, x, y),
    vertices(vector<Vertex*>()),
    pParent(nullptr)
{ }

bool Bag::ContainsVertex(Vertex* pV) const {
    for (int i=0; i<this->vertices.size(); ++i)
        if (this->vertices[i] == pV)
            return true;
    return false;
}

bool Bag::ContainsEdge(Edge* pE) const {
    // A bag 'contains an edge' if it contains both endpoints.
    bool a = false, b = false;
    for (int i=0; i<this->vertices.size(); ++i) {
        if (this->vertices[i] == pE->pA) {
            if (b)
                return true;
            a = true;
        }
        else if (this->vertices[i] == pE->pB) {
            if (a)
                return true;
            b = true;
        }
    }
    return false;
}

void Bag::SetParentsRecursive(Bag* pParent, bool adjustCoordinates) {
    // Update all bags recursively (the parent, the coordinates (optinally) and the order (vid 0 always in front))
    int diameter = 120;
    this->pParent = pParent;
    // Make sure the depot vertex is at the front of the list
    for (int i=1; i<this->vertices.size(); ++i) {
        if (isDepot(this->vertices[i])) {
            Vertex* pTemp = this->vertices[i];
            this->vertices[i] = this->vertices[0];
            this->vertices[0] = pTemp;
            break;
        }
    }
    // Adjust coords and recurse
    for (int j=0; j<this->edges.size(); ++j) {
        Bag* pBag = dynamic_cast<Bag*>(this->edges[j]->Other(*this));
        if (this->pParent == pBag)
            continue;
        if (adjustCoordinates) {
            pBag->y = this->y + diameter;
            pBag->x = this->x + (j - this->edges.size() / 2) * diameter;
        }
        pBag->SetParentsRecursive(this, adjustCoordinates);
    }
}

void Bag::TrimBagsRecursive(TreeDecomposition* pTD) {
    // First go in recursion (we won't want to delete bags first and then do something with them do we?)
    for (int j=this->edges.size()-1; j>=0; --j) {
        Bag* pBag = dynamic_cast<Bag*>(this->edges[j]->Other(*this));
        if (this->pParent != pBag)
            pBag->TrimBagsRecursive(pTD);
    }

    // Then check if this bag is a subset of its parent
    if (this->pParent != nullptr) {
        vector<Vertex*> v = this->pParent->vertices;
        bool noBreak = true;
        for (int i=0; i<this->vertices.size(); ++i) {
            if (find(v.begin(), v.end(), this->vertices[i]) == v.end()) {
                noBreak = false;
                break;
            }
        }
        if (noBreak) {
            // So it is a subset of its parent
            for (int j=0; j<this->edges.size(); ++j) {
                shared_ptr<Edge> pE = this->edges[j];
                // Remove it as child from its parent
                if (pE->Other(*this) == this->pParent) {
                    this->pParent->RemoveEdgeTo(this);
                    continue;
                }
                // Make all its children children of its parent
                Bag* pChildBag = dynamic_cast<Bag*>(pE->Other(*this));
                pChildBag->pParent = this->pParent;
                if (pE->pA == this)
                    pE->pA = this->pParent;
                else if (pE->pB == this)
                    pE->pB = this->pParent;
                // Add edges to the parent
                this->pParent->edges.push_back(pE);
            }
            // Delete the unnescessary bag
            for (int i=0; i<pTD->vertices.size(); ++i) {
                if (pTD->vertices[i].get() == this) {
                    pTD->vertices.erase(pTD->vertices.begin() + i);
                    return; // From here on 'this' is a pointer to a non-existing bag, so better return quickly.
                }
            }
        }
    }
}

vector<Edge*> Bag::GetBagEdges() const {
    // Get all edges corresponding to a bag (meaning all the edges of which both endpoints are vertices in this bag)
    vector<Edge*> Yi;
    for (int i=0; i<this->vertices.size(); ++i) {
        Vertex* pV = this->vertices[i];
        for (int j=0; j<pV->edges.size(); ++j) {
            Edge* pE = pV->edges[j].get();
            if (!this->ContainsEdge(pE))
                continue;
            if (pV->vid < pE->Other(*pV)->vid)
                Yi.push_back(pE);
        }
    }
    auto sortLambda = [&](Edge* pEdgeA, Edge* pEdgeB) {
        // Compare the costs of the edges
        return pEdgeA->Cost < pEdgeB->Cost;
    };
    sort(Yi.begin(), Yi.end(), sortLambda);
    if (Yi.size() > sizeof(int) * 8) {
        cout << "ASSERTION ERROR (getBagEdges): There are more edges in bag X" << this->vid << " then there are bits in an int: " << sizeof(int) << " bytes" << endl;
        assert(false);
    }
    return Yi;
}
