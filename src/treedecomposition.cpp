#include "treedecomposition.h"

#include <algorithm>
#include "utils.h"
#include <iostream>
using namespace std;

//
//  Tree decomposition
//
TreeDecomposition::TreeDecomposition(Graph* originalGraph)
    :pRoot(nullptr),
    pOriginalGraph(unique_ptr<Graph>(originalGraph))
{ }

TreeDecomposition::~TreeDecomposition()
{ }

bool TreeDecomposition::ReadFileLine(int& rState, string line) {
    // Handle one line of a file (note that the in place editing of the string in trim is a good thing).
    int startVid = 1; // TODO, THINK OF THIS ONE
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
        unique_ptr<Bag> pBag = unique_ptr<Bag>(new Bag(stoi(l[0]) - startVid, stoi(l[1]), stoi(l[2])));
        for (unsigned int i=3; i<l.size(); ++i) {
            // Add a specific vertex to a bag.
            pBag->vertices.push_back( this->pOriginalGraph->vertices[stoi(l[i]) - startVid].get() );
        }
        // Add the bag to the tree
        this->vertices.push_back(move(pBag));
        return true;
    }
    if (rState == 4) {
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
    for (unsigned int i=0; i<this->vertices.size(); ++i) {
        Bag* pBag = dynamic_cast<Bag*>(this->vertices[i].get());
        s += to_string(pBag->vid + startVid) + " " + to_string(pBag->x) + " " + to_string(pBag->y);
        for (unsigned int j=0; j<pBag->vertices.size(); ++j) {
            s += " " + to_string(pBag->vertices[j]->vid + startVid);
        }
        s += "\n";
    }
    s += "BAG_EDGE_SECTION\n";
    for (unsigned int i=0; i<this->vertices.size(); ++i) {
        Vertex* pBag = this->vertices[i].get();
        for (unsigned int j=0; j<pBag->edges.size(); ++j) {
            shared_ptr<Edge> pE = pBag->edges[j];
            if (pBag->vid < pE->Other(*pBag)->vid)
                s += to_string(pE->pA->vid + startVid) + " " + to_string(pE->pB->vid + startVid) + "\n";
        }
    }
    return s;
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

void TreeDecomposition::MinimumDegree() {
    // Check if the treedecompoistion is empty
    if (this->vertices.size() != 0 || this->pRoot != nullptr) {
        cout << "ERROR!!! - MinimumDegree called on a non empty tree decomposition" << endl;
        return;
    }
    // Prepare to create the tree decomposition
    const Graph& graph = *this->pOriginalGraph;
    vector<int> vertexList = vector<int>(graph.vertices.size());
    for (unsigned int i=0; i<vertexList.size(); ++i) {
        vertexList[i] = i;
    }
    auto sortLambda = [&](int vidA, int vidB) {
        // Compare the degrees of the vertices
        return graph.vertices[vidA]->edges.size() < graph.vertices[vidB]->edges.size();
    };
    sort(vertexList.begin(), vertexList.end(), sortLambda);
    vector<int> edgeList = vector<int>();

    // Create a tree decomposition (with wrong vids)
    this->permutationToTreeDecomposition(vertexList, edgeList);

    // Add edges
    Vertex* pA;
    Vertex* pB;
    for (unsigned int i=0; i<edgeList.size(); i+=2) {
        for (unsigned int j=0; j<this->vertices.size(); ++j) {
            Vertex* pBag = this->vertices[j].get();
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
    for (unsigned int i=0; i<this->vertices.size(); ++i) {
        this->vertices[i]->vid = i;
    }

    // Root the tree and give the bags some nice coordinates so that it actually draws like a tree with the root on top.
    this->CreateRoot(true);
}

/*void TreeDecomposition::permutationToTreeDecompositionORIGINAL(const vector<int>& vertexList, unsigned int recursionIdx, vector<int>& rEdgeList) {
    // Algorithm 2 from the paper titled "tw computations upper bounds" by B & K 2010.
    // vertexList:   The vid's sorted in order of smallest degree,
    // recursionIdx: The current vertexList should be [recIdx:], for performance reasons the list is passed intact as const reference - default: 0,
    // rEdgeList:    This list will be filled with int-pairs that should become the edges in the final tree decomposition
    //               (but the bags might not be created yet) - default: [].
    // Note: correct the vid's from each bag later (once the edges in the edge list are added)
    Vertex* pV = this->pOriginalGraph->vertices[vertexList[recursionIdx]].get();
    unique_ptr<Bag> pTheBag = unique_ptr<Bag>(new Bag(pV->vid, 240, 60 + 120 * recursionIdx));
    Bag* pBag = pTheBag.get();
    this->vertices.push_back(move(pTheBag));

    // Base case
    if (vertexList.size() - 1 == recursionIdx) {
        pBag->vertices.push_back(pV);
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
}*/
void TreeDecomposition::permutationToTreeDecomposition(const vector<int>& vertexList, vector<int>& rEdgeList) {
    // vertexList:   The vid's sorted in order of smallest degree,
    // rEdgeList:    This list will be filled with int-pairs that should become the edges in the final tree decomposition
    //               (but the bags might not be created yet) - provide it with an empty list.
    // Note: correct the vid's from each bag later; once the edges in the edge list are added
    unique_ptr<Graph> pGraphCopy = this->pOriginalGraph->DeepCopy();

    for (unsigned int i = 0; i<vertexList.size(); ++i) {
        Vertex* pV = this->pOriginalGraph->vertices[vertexList[i]].get();
        Vertex* pVCopy = pGraphCopy->vertices[pV->vid].get();
        unique_ptr<Bag> pUniqueBag = unique_ptr<Bag>(new Bag(pV->vid, 240, 60 + 120 * i));
        Bag* pBag = pUniqueBag.get();
        this->vertices.push_back(move(pUniqueBag));

        // Add the neighbourhood (in vertexList) of pV to the bag
        pBag->vertices.push_back(pV);
        for (unsigned int j=i + 1; j<vertexList.size(); ++j)
            for (unsigned int k=0; k<pVCopy->edges.size(); ++k)
                if (pVCopy->edges[k]->Other(*pVCopy)->vid == vertexList[j]) {
                    // We found a vertex that is both a 'neighbour' of pV and still in the vertex list - so add it
                    pBag->vertices.push_back(this->pOriginalGraph->vertices[ pVCopy->edges[k]->Other(*pVCopy)->vid ].get());
                    break;
                }

        // Clique-ify the neighbourhood of this vertex.
        for (unsigned int a=0; a<pBag->vertices.size(); ++a) {
            Vertex* pACopy = pGraphCopy->vertices[pBag->vertices[a]->vid].get();
            for (unsigned int b=a; b<pBag->vertices.size(); ++b) {
                Vertex* pBCopy = pGraphCopy->vertices[pBag->vertices[b]->vid].get();
                if (!pACopy->IsConnectedTo(pBCopy)) {
                    // Ok, a and b are not connected - add the edge (to the copied graph ofcourse - we're not modding the original graph)
                    shared_ptr<Edge> pECopy = shared_ptr<Edge>(new Edge(pACopy, pBCopy));
                    pACopy->edges.push_back(pECopy);
                    pBCopy->edges.push_back(pECopy);
                }
            }
        }

        // And find the right edge for in the tree decomposition
        for (unsigned int j=i + 1; j<vertexList.size(); ++j)
            for (unsigned int k=0; k<pVCopy->edges.size(); ++k)
                if (pVCopy->edges[k]->Other(*pVCopy)->vid == vertexList[j]) {
                    // We found the (future?) bag that our new bag should connect to - save it to add the edge later
                    rEdgeList.push_back(pV->vid);
                    rEdgeList.push_back(vertexList[j]);
                    // Break both for loops
                    j = vertexList.size();
                    break;
                }
    }
}


//
//  Bag
//
Bag::Bag(int vid, int x, int y)
    :Vertex::Vertex(vid, x, y),
    pParent(nullptr),
    vertices(vector<Vertex*>())
{ }

Bag::~Bag()
{ }

void Bag::SetParentsRecursive(Bag* pParent, bool adjustCoordinates) {
    // Update all bags recursively
    int diameter = 120;
    this->pParent = pParent;
    for (unsigned int i=0; i<this->edges.size(); ++i) {
        Bag* pBag = dynamic_cast<Bag*>(this->edges[i]->Other(*this));
        if (this->pParent == pBag)
            continue;
        if (adjustCoordinates) {
            pBag->y = this->y + diameter;
            pBag->x = this->x + (i - this->edges.size() / 2) * diameter;
        }
        pBag->SetParentsRecursive(this, adjustCoordinates);
    }
}
