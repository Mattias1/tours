#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <vector>
using namespace std;


//
// String split methods
// (Thank you SO: http://stackoverflow.com/questions/236129/split-a-string-in-c)
//
vector<string> &split(const string &s, char delim, vector<string> &elems);

vector<string> split(const string &s, char delim);


//
// String trim methods
// (Thank you SO: http://stackoverflow.com/questions/216823/whats-the-best-way-to-trim-stdstring)
// Note: looks like they edit the string in place
//
string &ltrim(string &rS);

string &rtrim(string &rS);

string &trim(string &rS);


#endif // UTILS_H
