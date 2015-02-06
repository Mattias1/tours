#include "graph.h"

#include "utils.h"
using namespace std;

//
//  Graph
//
Graph::Graph()
    :vertices(vector<shared_ptr<Vertex>>())
{ }

Graph::~Graph()
{ }

// This is a small function used by ReadFileLine.
bool comp(const string& line, const string& firstPart) {
    return line.substr(0, firstPart.size()) == firstPart;
}

bool Graph::ReadFileLine(int& rState, string line) {
    // Handle one line of a file (note that the in place editing of the string in trim is a good thing).
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
        this->vertices.push_back(shared_ptr<Vertex>(new Vertex(std::stoi(l[0]), std::stoi(l[1]), std::stoi(l[2]))));
        return true;
    }
    if (rState == 2) {
        // Add the edge to the edgelist of it's endpoints
        shared_ptr<Vertex> pA = this->vertices[stoi(l[0])];
        shared_ptr<Vertex> pB = this->vertices[stoi(l[1])];
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
    string s = "DIMENSION : " + to_string(this->vertices.size()) + "\n";
    s += "EDGE_WEIGHT_TYPE : EUC_2D\n";

    s += "NODE_COORD_SECTION\n";
    for (unsigned int i=0; i<this->vertices.size(); ++i) {
        shared_ptr<Vertex> pV = this->vertices[i];
        s += to_string(pV->vid) + " " + to_string(pV->x) + " " + to_string(pV->y) + "\n";
    }

    s += "EDGE_SECTION\n";
    for (unsigned int i=0; i<this->vertices.size(); ++i) {
        shared_ptr<Vertex> pV = this->vertices[i];
        for (unsigned int j=0; j<pV->edges.size(); ++j) {
            shared_ptr<Edge> pE = pV->edges[j];
            if (pV->vid < pE->Other(*pV)->vid)
                s += to_string(pE->pA->vid) + " " + to_string(pE->pB->vid) + " " + to_string(pE->Cost) + "\n";
        }
    }

    return s;
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

//
//  Edge
//
Edge::Edge(shared_ptr<Vertex> pA, shared_ptr<Vertex> pB)
    :pA(pA),
    pB(pB)
{
    // I don't think I can initialize this, so lets just assign t.
    this->updateEuclideanCost();
}

Edge::~Edge()
{ }

shared_ptr<Vertex> Edge::Other(const Vertex& v) {
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
