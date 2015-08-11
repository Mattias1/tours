#ifndef SAVINGS_H
#define SAVINGS_H

#include "graph.h"
#include <vector>
// #include <utility> // NECESSARY???

//
// The saving class
//
class Saving
{
    public:
        int I, J, Value;

        Saving(int i, int j, Graph& graph, vector<int> costDepot2i);
};

//
// The algotithm
//
int savings();

#endif // SAVINGS_H
