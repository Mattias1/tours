#ifndef SAVINGS_H
#define SAVINGS_H

#include "graph.h"
#include <vector>
using namespace std;

//
// The saving class
//
class Saving
{
    public:
        int I, J, Value;

        Saving(int i, int j, const Graph& graph, vector<int> costDepot2i);
};

ostream &operator<<(ostream &os, const Saving& s);

//
// The algotithm
//
int savings(Graph& rGraph);

#endif // SAVINGS_H
