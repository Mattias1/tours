tours
=====
This repo contains my masters thesis project inspired by the paper "Tour merging via Branch decomposition" by Cook and Seymour.
The idea is to create a set of promising tours using a hearistic (LKH) and then merge these tours and calculate the optimal tour in the resulting graph. This can be done efficiently because the merged tour (probably) has a small tree width.

In the project we use the LKH-2.0.7 (http://www.akira.ruc.dk/~keld/research/LKH/), they are not included in this repo. To get it to work, follow the following steps:
- Copy the header files to include/lkh-include/
- Copy the source files to src/lkh-src/
- Modify the src/lkh-src/LKHmain.c file by renaming the ```main``` function to ```main_lkh```.
- (Possibly) add the lkh-include/ and lkh-src/ directories to the code blocks compilers search directory.

