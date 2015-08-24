#ifndef UTILS_H
#define UTILS_H

#include "graph.h"
#include "dp_tsp.h"
#include "dp_vrp.h"
#include <string>
#include <vector>
#include <list>
using namespace std;


//
// Copy int-vector references
//
vector<int> duplicate(const vector<int>& lst);
vector<vector<int>> duplicate(const vector<vector<int>>& lst);
vector<MatchingEdge> duplicate(const vector<MatchingEdge>& lst);
vector<MatchingEdge*> duplicate(const vector<MatchingEdge*>& lst);
vector<vector<MatchingEdge*>> duplicate(const vector<vector<MatchingEdge*>>& lst);
vector<Edge*> duplicate(const vector<Edge*>& lst);

//
// Pointerize
//
vector<MatchingEdge*> pointerize(vector<MatchingEdge>& lst);

//
// Flatten a vector of vectors
//
vector<int> flatten(const vector<vector<int>>& lst);
vector<MatchingEdge*> flatten(const vector<vector<MatchingEdge*>>& lst);

//
// Push back an entire list
//
void pushBackList(vector<Edge*>* pOriginalList, const vector<Edge*>& listToAdd);
void pushBackList(vector<pair<int, int>>* pOriginalList, const vector<pair<int, int>>& listToAdd);


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
string join(const vector<MatchingEdge*>& v, char delim);
string join(const vector<MatchingEdge>& v, char delim);


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
// (and: http://stackoverflow.com/questions/5932391/determining-if-a-string-is-a-double)
//
bool isInt(const string& s, bool allowNegative=false);

double isDouble(const string& s);


//
// Some debug helpers
//
string dbg(const string& s, int i);
string dbg(const vector<int>& v);
string dbg(const vector<vector<int>>& v);
string dbg(const vector<vector<vector<int>>>& v);
string dbg(const vector<MatchingEdge>& matchings);
string dbg(const vector<vector<MatchingEdge>>& matchings);
string dbg(const vector<MatchingEdge*>& matchings);
string dbg(const vector<vector<MatchingEdge*>>& matchings);
string dbg(const vector<vector<pair<int, int>>>& pairs);
string dbg(const vector<Edge*>& edges);
string dbg(const vector<Edge*>* const pEdges);
string dbg(const vector<Vertex*>& vertices);
string dbg(const vector<pair<int, list<int>>*> paths);
string dbg(const vector<bool> v);
string dbg(const vector<vector<bool>>& v);


//
// stoi and to_string workaround
// (Thanks to: http://www.cplusplus.com/forum/beginner/120836/)
// (And to: http://stackoverflow.com/questions/12975341/to-string-is-not-a-member-of-std-says-so-g)
//
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
int stoi(const string& str, size_t* pos = 0, int base = 10);

string to_string(int n);
#endif // defined

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

#endif // UTILS_H
