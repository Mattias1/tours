#ifndef GRAPH_H
#define GRAPH_H

#include <cmath>
#include <memory>
#include <string>
#include <vector>
using namespace std;


// Forward declaration, because edge is going to need vertices (but only using (smart) pointers)
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
        virtual ~Edge();

        Vertex* Other(const Vertex& v);

    private:
        void updateEuclideanCost();
};


class Vertex
{
    friend class Graph;
    friend class Edge;
    friend class TreeDecomposition;
    friend class Bag;

    public:
        int getX() { return this->x; }
        int getY() { return this->y; }
        int getVid() { return this->vid; }
        const vector<shared_ptr<Edge>>& getEdges() { return this->edges; }

        Vertex(int vid, int x, int y);
        virtual ~Vertex();

        bool IsConnectedTo(Vertex* other);

    private:
        int vid, x, y;
        vector<shared_ptr<Edge>> edges;
};


class Graph
{
    friend class TreeDecomposition;

    public:
        string name;
        vector<unique_ptr<Vertex>> vertices;

        Graph();
        virtual ~Graph();

        virtual bool ReadFileLine(int& rState, string line);
        virtual string ToFileString() const;

        virtual unique_ptr<Graph> DeepCopy();
};

#endif // GRAPH_H
