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
        const shared_ptr<Bag> getParent() { return this->pParent; }
        const vector<shared_ptr<Vertex>> getVertices() { return this->vertices; }

        Bag(int vid, int x, int y);
        virtual ~Bag();

        void SetParentsRecursive(shared_ptr<Bag> pParent);

    private:
        shared_ptr<Bag> pParent;
        vector<shared_ptr<Vertex>> vertices;
};


class TreeDecomposition : public Graph
{
    public:
        const shared_ptr<Bag> getRoot() { return this->pRoot; }
        const shared_ptr<Graph> getOriginalGraph() { return this->pOriginalGraph; }

        TreeDecomposition(shared_ptr<Graph> originalGraph);
        virtual ~TreeDecomposition();

        virtual bool ReadFileLine(int& rState, string line) override;
        virtual string ToFileString() const override;

        bool CreateRoot();
        bool CreateRoot(shared_ptr<Bag> pRoot);

        static shared_ptr<TreeDecomposition> MinimumDegree(shared_ptr<Graph> pGraph);

    private:
        shared_ptr<Bag> pRoot;
        shared_ptr<Graph> pOriginalGraph;

        void permutationToTreeDecomposition(const vector<int>& vertexList, unsigned int recursionIdx, vector<int>& rEdgeList);
};

#endif // TREEDECOMPOSITION_H
