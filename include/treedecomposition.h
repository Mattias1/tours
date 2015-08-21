#ifndef TREEDECOMPOSITION_H
#define TREEDECOMPOSITION_H

#include <graph.h>
#include <vector>
using namespace std;


// Forward declaration, because Bag is going to need it (but only using pointers)
class TreeDecomposition;

// Import in this order, as they all depend on each other.
class Bag : public Vertex
{
    friend class TreeDecomposition;

    public:
        const Bag* getParent() const { return this->pParent; }
        vector<Vertex*> vertices;

        Bag(int vid, int x, int y);

        bool ContainsVertex(Vertex* pV) const;
        bool ContainsEdge(Edge* pE) const;

        void SetParentsRecursive(Bag* pParent, bool adjustCoordinates=false);

        void TrimBagsRecursive(TreeDecomposition* pTD);

        vector<Edge*> GetBagEdges() const;

    private:
        Bag* pParent;
};


class TreeDecomposition : public Graph
{
    public:
        const Bag* getRoot() const { return this->pRoot; }
        const Graph* getOriginalGraph() const { return this->pOriginalGraph.get(); }

        TreeDecomposition(Graph* originalGraph);

        virtual bool ReadFileLine(int& rState, string line) override;
        virtual string ToFileString() const override;

        int GetTreeWidth() const;

        bool CreateRoot(bool adjustCoordinates=false);
        bool CreateRoot(Bag* pRoot, bool adjustCoordinates=false);

        void MinimumDegree(bool depotInAllBags = false);

    private:
        Bag* pRoot;
        unique_ptr<Graph> pOriginalGraph;

        void permutationToTreeDecomposition(bool depotInAllBags);
};

#endif // TREEDECOMPOSITION_H
