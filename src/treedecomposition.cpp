#include "treedecomposition.h"

#include <algorithm>
#include "utils.h"
using namespace std;

//
//  Tree decomposition
//
TreeDecomposition::TreeDecomposition(shared_ptr<Graph> originalGraph)
    :pRoot(nullptr),
    pOriginalGraph(originalGraph)
{ }

TreeDecomposition::~TreeDecomposition()
{ }

bool TreeDecomposition::ReadFileLine(int& rState, string line) {
    // Handle one line of a file (note that the in place editing of the string in trim is a good thing).
    vector <string> l = split(trim(line), ' ');
    // Important file parameters
    if (comp(line, "NAME : ")) {
        this->name = l[2];
        return true;
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
        shared_ptr<Bag> pBag = shared_ptr<Bag>(new Bag(stoi(l[0]), stoi(l[1]), stoi(l[2])));
        for (unsigned int i=3; i<l.size(); ++i) {
            // Add a specific vertex to a bag.
            pBag->vertices.push_back(shared_ptr<Vertex>( this->pOriginalGraph->vertices[ stoi(l[i]) ] ));
        }
        // Add the bag to the tree
        this->vertices.push_back(pBag);
        return true;
    }
    if (rState == 4) {
        // Add the edge to the edgelist of it's endpoints
        shared_ptr<Vertex> pA = shared_ptr<Vertex>(this->vertices[stoi(l[0])]);
        shared_ptr<Vertex> pB = shared_ptr<Vertex>(this->vertices[stoi(l[1])]);
        shared_ptr<Edge> pE = shared_ptr<Edge>(new Edge(pA, pB));
        pA->edges.push_back(pE);
        pB->edges.push_back(pE);
        return true;
    }

    return false;
}

string TreeDecomposition::ToFileString() const {
    // This is supposed to be placed after the normal graph.ToFileString method.
    if (this->vertices.size() == 0)
        return "";
    string s = "BAG_COORD_SECTION\n";
    for (unsigned int i=0; i<this->vertices.size(); ++i) {
        shared_ptr<Bag> pBag = dynamic_pointer_cast<Bag>(this->vertices[i]);
        s += to_string(pBag->vid) + " " + to_string(pBag->x) + " " + to_string(pBag->y);
        for (unsigned int j=0; j<pBag->vertices.size(); ++j) {
            s += " " + to_string(pBag->vertices[j]->vid);
        }
        s += "\n";
    }
    s += "BAG_EDGE_SECTION\n";
    for (unsigned int i=0; i<this->vertices.size(); ++i) {
        shared_ptr<Vertex> pBag = this->vertices[i];
        for (unsigned int j=0; j<pBag->edges.size(); ++j) {
            shared_ptr<Edge> pE = pBag->edges[j];
            if (pBag->vid < pE->Other(*pBag)->vid)
                s += to_string(pE->pA->vid) + " " + to_string(pE->pB->vid) + "\n";
        }
    }
    return s;
}

bool TreeDecomposition::CreateRoot() {
    if (this->vertices.size() <= 0)
        return false;
    return this->CreateRoot(dynamic_pointer_cast<Bag>(this->vertices[0]));
}
bool TreeDecomposition::CreateRoot(shared_ptr<Bag> pRoot) {
    if (pRoot == nullptr)
        return false;
    pRoot->SetParentsRecursive(nullptr);
    this->pRoot = pRoot;
    return true;
}

shared_ptr<TreeDecomposition> TreeDecomposition::MinimumDegree(shared_ptr<Graph> pGraph) {
    // Prepare to create the tree decomposition
    shared_ptr<TreeDecomposition> pTD = shared_ptr<TreeDecomposition>(new TreeDecomposition(pGraph));
    vector<int> vertexList = vector<int>(pGraph->vertices.size());
    for (unsigned int i=0; i<vertexList.size(); ++i) {
        vertexList[i] = i;
    }
    auto sortLambda = [=](int vidA, int vidB) {
        // Compare the degrees of the vertices
        return pGraph->vertices[vidA]->edges.size() < pGraph->vertices[vidB]->edges.size();
    };
    sort(vertexList.begin(), vertexList.end(), sortLambda);
    vector<int> edgeList = vector<int>();

    // Create a tree decomposition (with wrong vids)
    pTD->permutationToTreeDecomposition(vertexList, 0, edgeList);

    // Add edges
    shared_ptr<Vertex> pA;
    shared_ptr<Vertex> pB;
    for (unsigned int i=0; i<edgeList.size(); i+=2) {
        for (unsigned int j=0; j<pTD->vertices.size(); ++j) {
            shared_ptr<Vertex> pBag = pTD->vertices[j];
            if (pBag->vid == edgeList[i])
                pA = pBag;
            if (pBag->vid == edgeList[i + 1])
                pB = pBag;
        }
        shared_ptr<Edge> pE = shared_ptr<Edge>(new Edge(pA, pB));
        pA->edges.push_back(pE);
        pB->edges.push_back(pE);
    }

    // Fix vids
    for (unsigned int i=0; i<pTD->vertices.size(); ++i) {
        pTD->vertices[i]->vid = i;
    }

    return pTD;
}

void TreeDecomposition::permutationToTreeDecomposition(const vector<int>& vertexList, unsigned int recursionIdx, vector<int>& rEdgeList) {
    // Algorithm 2 from the paper titled "tw computations upper bounds" by B & K 2010.
    // vertexList:   The vid's sorted in order of smallest degree,
    // recursionIdx: The current vertexList should be [recIdx:], for performance reasons the list is passed intact as const reference - default: 0,
    // rEdgeList:    This list will be filled with int-pairs that should become the edges in the final tree decomposition
    //               (but the bags might not be created yet) - default: [].
    // TODO: correct the vid's from each bag later (including all edges etc)
    shared_ptr<Vertex> pV = this->pOriginalGraph->vertices[vertexList[recursionIdx]];
    shared_ptr<Bag> pBag = shared_ptr<Bag>(new Bag(pV->vid, 30, 30 + 30 * recursionIdx)); // Coordinates are arbitrary

    // Base case
    if (vertexList.size() - 1 == recursionIdx) {
        pBag->vertices.push_back(pV);
        this->vertices.push_back(pBag);
        return;
    }

    // Normal case (recurse)
    this->permutationToTreeDecomposition(vertexList, recursionIdx + 1, rEdgeList);
    // Add the neighbourhood (in vertexList) of pV to the bag
    pBag->vertices.push_back(pV);
    for (unsigned int i=recursionIdx + 1; i<vertexList.size(); ++i)
        for (unsigned int j=0; j<pV->edges.size(); ++j)
            if (pV->edges[j]->Other(*pV)->vid == vertexList[i]) {
                // We found a vertex that is both a neighbour of pV and still in the vertex list - so add it
                pBag->vertices.push_back(pV->edges[j]->Other(*pV));
                break;
            }
    // And find the right edge
    for (unsigned int i=recursionIdx + 1; i<vertexList.size(); ++i)
        for (unsigned int j=0; j<pV->edges.size(); ++j)
            if (pV->edges[j]->Other(*pV)->vid == vertexList[i]) {
                // We found the (future?) bag that our new bag should connect to - save it to add the edge later
                rEdgeList.push_back(pV->vid);
                rEdgeList.push_back(vertexList[i]);
                return;
            }
}


//
//  Bag
//
Bag::Bag(int vid, int x, int y)
    :Vertex::Vertex(vid, x, y),
    pParent(nullptr),
    vertices(vector<shared_ptr<Vertex>>())
{ }

Bag::~Bag()
{ }

void Bag::SetParentsRecursive(shared_ptr<Bag> pParent) {
    // Update all bags recursively
    this->pParent = pParent;
    for (unsigned int i=0; i<this->edges.size(); ++i) {
        shared_ptr<Bag> pBag = dynamic_pointer_cast<Bag>(this->edges[i]->Other(*this));
        if (this->pParent == pBag)
            continue;
        pBag->SetParentsRecursive(shared_ptr<Bag>(this));
    }
}
