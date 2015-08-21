tours
=====
This repo contains my masters thesis project inspired by the paper "Tour merging via Branch decomposition" by Cook and Seymour.
The goal is to solve the TSP or VRP using a hybrid between heuristics and optimal algorithms.
The idea is to create a set of promising tours using a hearistic (LKH / Savings / Sweep) and then merge these tours and calculate the optimal tour in the resulting graph. This can be done efficiently because the merged graph (probably) has a small treewidth.

In the project we use LKH-2.0.7 for the heuristics. To get it to work, follow these steps:
- Download the source files (http://www.akira.ruc.dk/~keld/research/LKH/).
- Copy the header files to _include/lkh-include/_.
- Copy the source files to _src/lkh-src/_.
- Modify the _src/lkh-src/LKHmain.c_ file by renaming the `main` function to `main_lkh`.
- (Possibly) add the _lkh-include/_ and _lkh-src/_ directories to the code blocks compilers search directory.
- Include the LKH binary in a folder named _LKH-2.0.7/_.

