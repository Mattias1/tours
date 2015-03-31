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
        const Bag* getParent() const { return this->pParent; }
        vector<Vertex*> vertices;

        Bag(int vid, int x, int y);
        virtual ~Bag();

        bool ContainsVertex(Vertex* pV) const;
        bool ContainsEdge(Edge* pE) const;

        void SetParentsRecursive(Bag* pParent, bool adjustCoordinates=false);

    private:
        Bag* pParent;
};


class TreeDecomposition : public Graph
{
    public:
        const Bag* getRoot() const { return this->pRoot; }
        const Graph* getOriginalGraph() const { return this->pOriginalGraph.get(); }

        TreeDecomposition(Graph* originalGraph);
        virtual ~TreeDecomposition();

        virtual bool ReadFileLine(int& rState, string line) override;
        virtual string ToFileString() const override;

        int GetTreeWidth() const;

        bool CreateRoot(bool adjustCoordinates=false);
        bool CreateRoot(Bag* pRoot, bool adjustCoordinates=false);

        void MinimumDegree();

    private:
        Bag* pRoot;
        unique_ptr<Graph> pOriginalGraph;

        void permutationToTreeDecomposition(const vector<int>& vertexList, vector<int>& rEdgeList);
};

#endif // TREEDECOMPOSITION_H
