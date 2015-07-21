#include "graph.h"

#include "utils.h"
#include <iostream>
using namespace std;

//
//  Graph
//
Graph::Graph()
{ }
Graph::~Graph()
{ }

// This is a small function used by ReadFileLine.
bool comp(const string& line, const string& firstPart) {
    return line.substr(0, firstPart.size()) == firstPart;
}

bool Graph::ReadFileLine(int& rState, string line) {
    // Handle one line of a file (note that the in place editing of the string in trim is a good thing).
    int startVid = 1; // Apparently ppl like to have their lists 1-based, rather than 0-based.
    vector <string> l = split(trim(line), ' ');
    // Important file parameters
    if (comp(line, "NAME : ")) {
        this->name = l[2];
        return true;
    }
    if (comp(line, "CAPACITY : ")) {
        this->capacity = stoi(l[2]);
        return true;
    }
    if (comp(line, "TRUCKS : ")) {
        this->trucks = stoi(l[2]);
        return true;
    }
    if (comp(line, "EOF")) {
        rState = 0;
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
    if (comp(line, "DEMAND_SECTION")) {
        rState = 5;
        return true;
    }
    if (comp(line, "DEPOT_SECTION")) {
        rState = 6;
        return true;
    }
    if (comp(line, "BAG_COORD_SECTION") || comp(line, "BAG_EDGE_SECTION")) {
        return false;
    }

    // Add vertices, edges, bags or bag edges
    if (rState == 1) {
        if (l.size() < 3 || l.size() > 4)
            cout << "ERROR: The Graph ReadFileLine expects a vertex, but it doesn't get three or four strings in the first place." << endl;
        // Add the vertex to the graph
        int demand = l.size() == 4? stoi(l[3]) : 0;
        this->vertices.push_back(unique_ptr<Vertex>(new Vertex(stoi(l[0]) - startVid, stoi(l[1]), stoi(l[2]), demand)));
        return true;
    }
    if (rState == 2) {
        if (l.size() < 2 || l.size() > 3)
            cout << "ERROR: The Graph ReadFileLine expects an edge, but it doesn't get two or three strings in the first place." << endl;
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
    if (rState == 5) {
        if (l.size() != 2)
            cout << "ERROR: We expect two integers (vid demand) for the demand_section." << endl;
        // Change the demand for the vertex
        this->vertices[stoi(l[0]) - startVid]->demand = stoi(l[1]);
        return true;
    }
    if (rState == 6) {
        // Well, I kinda assume the depot is just the first vertex in the list. Let's see how far we get with that.
        // This whole section is there only because some VRP variants have multiple depots, but I'm ignoring those.
        // If someone has it's depot in a weird place, I'd have to just swap it with the first vertex (and fix vid's). But for now we'll keep it at this.
        if (l.size() != 1 && stoi(l[0]) != startVid && stoi(l[0]) != -1)
            cout << "WHOOPS, THE DEPOT IS NOT THE FIRST VERTEX (or maybe another error, like it not being a single integer?)" << endl;
        if (this->vertices[0]->demand != 0)
            cout << "REMARKABLE: The demand of the depot is not 0." << endl;
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
    for (int i=0; i<this->vertices.size(); ++i) {
        Vertex* pV = this->vertices[i].get();
        s += to_string(pV->vid + startVid) + " " + to_string(pV->x) + " " + to_string(pV->y);
        if (pV->demand > 0)
            s += " " + to_string(pV->demand);
        s += "\n";
    }

    s += "EDGE_SECTION\n";
    for (int i=0; i<this->vertices.size(); ++i) {
        Vertex* pV = this->vertices[i].get();
        for (int j=0; j<pV->edges.size(); ++j) {
            Edge* pE = pV->edges[j].get();
            if (pV->vid < pE->Other(*pV)->vid)
                s += to_string(pE->pA->vid + startVid) + " " + to_string(pE->pB->vid + startVid) + " " + to_string(pE->Cost) + "\n";
        }
    }

    return s;
}

bool Graph::AddTourFromFile(const vector<int>& vids) {
    // Add all edges from the cycle
    int startVid = 1;
    bool result = false;
    for (int i=0; i<vids.size() - 1; ++i) {
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

unique_ptr<Graph> Graph::DeepCopy() const {
    // Create a deep copy of the graph
    unique_ptr<Graph> pGraph = unique_ptr<Graph>(new Graph());
    pGraph->name = this->name + " (copy)";

    // Add all vertices
    for (int i=0; i<this->vertices.size(); ++i) {
        const Vertex& v = *this->vertices[i];
        pGraph->vertices.push_back( unique_ptr<Vertex>(new Vertex(v.vid, v.x, v.y, v.demand)) );
    }

    // Add all edges
    for (int i=0; i<this->vertices.size(); ++i) {
        const Vertex& v = *this->vertices[i];
        Vertex* pA = pGraph->vertices[i].get();
        for (int j=0; j<v.edges.size(); ++j) {
            Vertex* pB = pGraph->vertices[ v.edges[j]->Other(v)->vid ].get();
            shared_ptr<Edge> pE = shared_ptr<Edge>(new Edge(pA, pB));
            pA->edges.push_back(pE);
            pB->edges.push_back(pE);
        }
    }

    // This'll probably be optimized by the compiler to just copy once (this creation) and not again at the return call.
    return move(pGraph);
}

unique_ptr<Graph> Graph::CreateTourGraph(vector<Edge*> tour) const {
    // Create a (deep) copy of the graph's vertices, and add a (deep) copy of the tour edges only
    unique_ptr<Graph> pGraph = unique_ptr<Graph>(new Graph());
    pGraph->name = this->name + " (tour copy)";

    // Add all vertices
    for (int i=0; i<this->vertices.size(); ++i) {
        const Vertex& v = *this->vertices[i];
        pGraph->vertices.push_back( unique_ptr<Vertex>(new Vertex(v.vid, v.x, v.y)) );
    }

    // Add all edges
    for (int i=0; i<tour.size(); ++i) {
        Vertex* pA = pGraph->vertices[tour[i]->pA->vid].get();
        Vertex* pB = pGraph->vertices[tour[i]->pB->vid].get();
        shared_ptr<Edge> pE = shared_ptr<Edge>(new Edge(pA, pB));
        pA->edges.push_back(pE);
        pB->edges.push_back(pE);
    }

    return move(pGraph);
}
unique_ptr<Graph> Graph::CreateTourGraph(vector<vector<Edge*>> tours) const {
    // Create a (deep) copy of the graph's vertices, and add a (deep) copy of the tour edges only
    unique_ptr<Graph> pGraph = unique_ptr<Graph>(new Graph());
    pGraph->name = this->name + " (tours copy)";

    // Add all vertices
    for (int i=0; i<this->vertices.size(); ++i) {
        const Vertex& v = *this->vertices[i];
        pGraph->vertices.push_back( unique_ptr<Vertex>(new Vertex(v.vid, v.x, v.y)) );
    }

    // Add all edges
    for (int j=0; j<tours.size(); ++j)
        for (int i=0; i<tours[j].size(); ++i) {
            Vertex* pA = pGraph->vertices[tours[j][i]->pA->vid].get();
            Vertex* pB = pGraph->vertices[tours[j][i]->pB->vid].get();
            shared_ptr<Edge> pE = shared_ptr<Edge>(new Edge(pA, pB));
            pA->edges.push_back(pE);
            pB->edges.push_back(pE);
        }

    return move(pGraph);
}

//
//  Vertex
//
Vertex::Vertex(int vid, int x, int y, int demand /*=0*/)
    :vid(vid),
    x(x),
    y(y),
    demand(demand)
{ }
Vertex::~Vertex()
{ }

bool Vertex::IsConnectedTo(Vertex* pOther) const {
    // Return whether or not this vertex is connected to another vertex (a vertex is considered to be connected to himself)
    if (this == pOther)
        return true;
    for (int i=0; i<this->edges.size(); ++i)
        if (this->edges[i]->Other(*this) == pOther)
            return true;
    return false;
}

bool Vertex::RemoveEdgeTo(Vertex* pOther) {
    // Removes the edge to the other vertex (if it doesn't exists, return false)
    // WARNING: DOES NOT REMOVE THE EDGE THE OTHER WAY AROUND!!! (from pOther to this)
    for (int j=0; j<this->edges.size(); ++j) {
        if (this->edges[j]->Other(*this) == pOther) {
            this->edges.erase(this->edges.begin() + j);
            cout << "DEBUG REMOVE EDGE TO - TRUE" << endl;
            return true;
        }
    }
    cout << "DEBUG REMOVE EDGE TO - FALSE" << endl;
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

Vertex* Edge::Other(const Vertex& v) const {
    if (v.vid == this->pA->vid)
        return this->pB;
    else if (v.vid == this->pB->vid)
        return this->pA;
    else
        return nullptr;
}

bool Edge::IsIncidentTo(Vertex* pV) const {
    return this->pA == pV || this->pB == pV;
}

void Edge::updateEuclideanCost() {
    int x = this->pA->x - this->pB->x;
    int y = this->pA->y - this->pB->y;
    this->Cost = sqrt(x*x + y*y);
}

ostream &operator<<(ostream &os, const Edge& edge) {
    return os << edge.pA->vid << "-" << edge.pB->vid;
    // return os << "(" << edge.pA->vid; << ", " << edge.pB->vid << ")";
}
