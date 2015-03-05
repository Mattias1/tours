#include "graph.h"

#include "utils.h"
#include <iostream>
using namespace std;

//
//  Graph
//
Graph::Graph()
    :vertices(vector<unique_ptr<Vertex>>())
{ }

Graph::~Graph()
{ }

// This is a small function used by ReadFileLine.
bool comp(const string& line, const string& firstPart) {
    return line.substr(0, firstPart.size()) == firstPart;
}

bool Graph::ReadFileLine(int& rState, string line) {
    // Handle one line of a file (note that the in place editing of the string in trim is a good thing).
    int startVid = 1;
    vector <string> l = split(trim(line), ' ');
    // Important file parameters
    if (comp(line, "NAME : ")) {
        this->name = l[2];
        return true;
    }
    // Vertices and edges
    if (comp(line, "NODE_COORD_SECTION")) {
        rState = 1;
        return true;
    }
    if (comp(line, "EDGE_SECTION")) {
        rState = 2;
        return true;
    }
    if (comp(line, "BAG_COORD_SECTION") || comp(line, "BAG_EDGE_SECTION")) {
        return false;
    }
    // Add vertices, edges, bags or bag edges
    if (rState == 1) {
        // Add the vertex to the graph
        this->vertices.push_back(unique_ptr<Vertex>(new Vertex(stoi(l[0]) - startVid, stoi(l[1]), stoi(l[2]))));
        return true;
    }
    if (rState == 2) {
        // Add the edge to the edgelist of it's endpoints
        Vertex* pA = this->vertices[stoi(l[0]) - startVid].get();
        Vertex* pB = this->vertices[stoi(l[1]) - startVid].get();
        /* int cost = 1;
           if (l.size() > 2)
               cost = stoi(l[2]);
        */
        shared_ptr<Edge> pE = shared_ptr<Edge>(new Edge(pA, pB));
        pA->edges.push_back(pE);
        pB->edges.push_back(pE);
        return true;
    }
    return false;
}

string Graph::ToFileString() const {
    // Create a string with the graph info in it
    int startVid = 1;
    string s = "DIMENSION : " + to_string(this->vertices.size()) + "\n";
    s += "EDGE_WEIGHT_TYPE : EUC_2D\n";

    s += "NODE_COORD_SECTION\n";
    for (unsigned int i=0; i<this->vertices.size(); ++i) {
        Vertex* pV = this->vertices[i].get();
        s += to_string(pV->vid + startVid) + " " + to_string(pV->x) + " " + to_string(pV->y) + "\n";
    }

    s += "EDGE_SECTION\n";
    for (unsigned int i=0; i<this->vertices.size(); ++i) {
        Vertex* pV = this->vertices[i].get();
        for (unsigned int j=0; j<pV->edges.size(); ++j) {
            Edge* pE = pV->edges[j].get();
            if (pV->vid < pE->Other(*pV)->vid)
                s += to_string(pE->pA->vid + startVid) + " " + to_string(pE->pB->vid + startVid) + " " + to_string(pE->Cost) + "\n";
        }
    }

    return s;
}

bool Graph::AddTourFromFile(const vector<unsigned int>& vids) {
    // Add all edges from the cycle
    int startVid = 1;
    bool result = false;
    for (unsigned int i=0; i<vids.size() - 1; ++i) {
        // Assert that the vertices exist
        if (vids[i] - startVid >= this->vertices.size() || vids[i + 1] - startVid >= this->vertices.size()) {
            cout << "ERROR, the vertex requested by the AddTourFromFile doesn't exist - " << vids[i] - startVid << ", " << vids[i + 1] - startVid << endl;
            return false;
        }
        // Add the actual edge (if it's not there already).
        Vertex* pA = this->vertices[vids[i] - startVid].get();
        Vertex* pB = this->vertices[vids[i + 1] - startVid].get();
        if (pA->IsConnectedTo(pB))
            continue;
        shared_ptr<Edge> pE = shared_ptr<Edge>(new Edge(pA, pB));
        pA->edges.push_back(pE);
        pB->edges.push_back(pE);
        result = true;
    }
    return result;
}

unique_ptr<Graph> Graph::DeepCopy() {
    // Create a deep copy of the graph
    unique_ptr<Graph> pGraph = unique_ptr<Graph>(new Graph());
    pGraph->name = this->name + " (copy)";

    // Add all vertices
    for (unsigned int i=0; i<this->vertices.size(); ++i) {
        const Vertex& v = *this->vertices[i];
        pGraph->vertices.push_back( unique_ptr<Vertex>(new Vertex(v.vid, v.x, v.y)) );
    }

    // Add all edges
    for (unsigned int i=0; i<this->vertices.size(); ++i) {
        const Vertex& v = *this->vertices[i];
        Vertex* pA = pGraph->vertices[i].get();
        for (unsigned int j=0; j<v.edges.size(); ++j) {
            Vertex* pB = pGraph->vertices[ v.edges[j]->Other(v)->vid ].get();
            shared_ptr<Edge> pE = shared_ptr<Edge>(new Edge(pA, pB));
            pA->edges.push_back(pE);
            pB->edges.push_back(pE);
        }
    }

    // This'll probably be optimized by the compiler to just copy once (this creation) and not again at the return call.
    return move(pGraph);
}

//
//  Vertex
//
Vertex::Vertex(int vid, int x, int y)
    :vid(vid),
    x(x),
    y(y),
    edges(vector<shared_ptr<Edge>>())
{ }

Vertex::~Vertex()
{ }

bool Vertex::IsConnectedTo(Vertex* other) {
    // Return whether or not this vertex is connected to another vertex (a vertex is considered to be connected to himself)
    if (this == other)
        return true;
    for (unsigned int i=0; i<this->edges.size(); ++i)
        if (this->edges[i]->Other(*this) == other)
            return true;
    return false;
}

//
//  Edge
//
Edge::Edge(Vertex* pA, Vertex* pB)
    :pA(pA),
    pB(pB)
{
    // I don't think I can initialize this, so lets just assign t.
    this->updateEuclideanCost();
}

Edge::~Edge()
{ }

Vertex* Edge::Other(const Vertex& v) {
    if (v.vid == this->pA->vid)
        return this->pB;
    else if (v.vid == this->pB->vid)
        return this->pA;
    else
        return nullptr;
}

void Edge::updateEuclideanCost() {
    int x = this->pA->x - this->pB->x;
    int y = this->pA->y - this->pB->y;
    this->Cost = sqrt(x*x + y*y);
}
