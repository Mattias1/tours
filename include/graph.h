#ifndef GRAPH_H
#define GRAPH_H

#include <cmath>
#include <memory>
#include <string>
#include <vector>
using namespace std;


// Forward declaration, because edge is going to need vertices (but only using pointers)
class Vertex;
// Header file declaration of small compare function, because it's also used in the tree decomposition file
bool comp(const string& line, const string& firstPart);

// Import in this order, as they all depend on each other.
class Edge
{
    friend class Graph;
    friend class Vertex;

    public:
        Vertex* pA;
        Vertex* pB;
        int Cost;

        Edge(Vertex* pA, Vertex* pB);

        Vertex* Other(const Vertex& v) const;

        bool IsIncidentTo(Vertex* pV) const;

    private:
        void updateEuclideanCost();
};

ostream &operator<<(ostream &os, const Edge& edge);


class Vertex
{
    friend class Graph;
    friend class Edge;
    friend class TreeDecomposition;
    friend class Bag;

    public:
        int vid, x, y, demand;
        vector<shared_ptr<Edge>> edges;

        Vertex(int vid, int x, int y, int demand = 0);
        virtual ~Vertex();

        bool IsConnectedTo(Vertex* other) const;

        bool RemoveEdgeTo(Vertex* other);
};


class Graph
{
    friend class TreeDecomposition;

    public:
        string name;
        int capacity, trucks;
        vector<unique_ptr<Vertex>> vertices;

        Graph();
        virtual ~Graph();

        virtual bool ReadFileLine(int& rState, string line);
        virtual string ToFileString() const;
        bool AddTourFromFile(const vector<int>& vids);

        virtual unique_ptr<Graph> DeepCopy() const;
        unique_ptr<Graph> CreateTourGraph(vector<Edge*> tour) const;
        unique_ptr<Graph> CreateTourGraph(vector<vector<Edge*>> tours) const;
};

#endif // GRAPH_H
