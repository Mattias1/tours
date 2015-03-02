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


//
// Check if string is an integer number
// (Thank you SO: http://stackoverflow.com/questions/4654636/how-to-determine-if-a-string-is-a-number-with-c)
//
bool isInt(const string& s, bool allowNegative=false);


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
