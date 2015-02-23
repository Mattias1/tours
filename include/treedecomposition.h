#ifndef TREEDECOMPOSITION_H
#define TREEDECOMPOSITION_H

#include <graph.h>
#include <vector>
using namespace std;


// Import in this order, as they all depend on eachother.
class Bag : public Vertex
{
    friend class TreeDecomposition;

    public:
        const Bag* getParent() { return this->pParent; }
        //const vector<unique_ptr<Vertex>> getVertices() { return this->vertices; }

        Bag(int vid, int x, int y);
        virtual ~Bag();

        void SetParentsRecursive(Bag* pParent, bool adjustCoordinates=false);

    private:
        Bag* pParent;
        vector<Vertex*> vertices;
};


class TreeDecomposition : public Graph
{
    public:
        const Bag* getRoot() { return this->pRoot; }
        const Graph* getOriginalGraph() { return this->pOriginalGraph.get(); }

        TreeDecomposition(Graph* originalGraph);
        virtual ~TreeDecomposition();

        virtual bool ReadFileLine(int& rState, string line) override;
        virtual string ToFileString() const override;

        bool CreateRoot(bool adjustCoordinates=false);
        bool CreateRoot(Bag* pRoot, bool adjustCoordinates=false);

        void MinimumDegree();

    private:
        Bag* pRoot;
        unique_ptr<Graph> pOriginalGraph;

        void permutationToTreeDecomposition(const vector<int>& vertexList, vector<int>& rEdgeList);
};

#endif // TREEDECOMPOSITION_H
