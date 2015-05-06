#ifndef UTILS_H
#define UTILS_H

#include "graph.h"
#include <string>
#include <vector>
using namespace std;


//
// Copy int-vector references
//
vector<int> duplicate(const vector<int>& lst);
vector<vector<int>> duplicate(const vector<vector<int>>& lst);
vector<Edge*> duplicate(const vector<Edge*>& lst);

//
// Flatten a vector of int-vectors
//
vector<int> flatten(const vector<vector<int>>& lst);

//
// Push back an entire list
//
void pushBackList(vector<Edge*>* pOriginalList, const vector<Edge*>& listToAdd);


//
// String split methods
// (Thank you SO: http://stackoverflow.com/questions/236129/split-a-string-in-c)
//
vector<string> &split(const string &s, char delim, vector<string> &elems);
vector<string> split(const string &s, char delim);

vector<int> &splitInt(const string &s, char delim, vector<int> &elems);
vector<int> splitInt(const string &s, char delim);

//
// String join method
// (Thank you SO: http://stackoverflow.com/questions/1430757/c-vector-to-string)
//
string join(const vector<int>& v, char delim);


//
// String trim methods
// (Thank you SO: http://stackoverflow.com/questions/216823/whats-the-best-way-to-trim-stdstring)
// Note: looks like they edit the string in place
//
string &ltrim(string &rS);
string &rtrim(string &rS);
string &trim(string &rS);


//
// Check if string is an integer number
// (Thank you SO: http://stackoverflow.com/questions/4654636/how-to-determine-if-a-string-is-a-number-with-c)
//
bool isInt(const string& s, bool allowNegative=false);


//
// Some debug helpers
//
string dbg(const string& s, unsigned int i);
string dbg(const vector<int>& v);
string dbg(const vector<vector<int>>& v);
string dbg(const vector<Edge*>& edges);
string dbg(vector<Edge*>* pEdges);
string dbg(const vector<Vertex*>& vertices);


//
// stoi and to_string workaround
// (Thanks to: http://www.cplusplus.com/forum/beginner/120836/)
// (And to: http://stackoverflow.com/questions/12975341/to-string-is-not-a-member-of-std-says-so-g)
//
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
int stoi(const string& str, size_t* pos = 0, int base = 10);

string to_string(int n);
#endif // defined

#endif // UTILS_H
