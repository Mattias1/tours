#ifndef SAVINGS_H
#define SAVINGS_H

#include "graph.h"
#include <vector>

//
// The saving class
//
class Saving
{
    public:
        int I, J, Value;

        Saving(int i, int j, const Graph& graph, vector<int> costDepot2i);
};

//
// The algotithm
//
int savings(Graph& rGraph);

#endif // SAVINGS_H
