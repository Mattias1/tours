#include "utils.h"

#include <algorithm>
#include <functional>
#include <cctype>
#include <locale>
#include <sstream>

using namespace std;

//
// String split methods
//
vector<string> &split(const string &s, char delim, vector<string> &elems) {
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

vector<string> split(const string &s, char delim) {
    vector<string> elems;
    split(s, delim, elems);
    return elems;
}

//
// String trim methods
//
string &ltrim(string &rS) {
    // trim from start
    rS.erase(rS.begin(), find_if(rS.begin(), rS.end(), not1(ptr_fun<int, int>(isspace))));
    return rS;
}

string &rtrim(string &rS) {
    // trim from end
    rS.erase(find_if(rS.rbegin(), rS.rend(), not1(ptr_fun<int, int>(isspace))).base(), rS.end());
    return rS;
}

string &trim(string &rS) {
    // trim from both ends
    return ltrim(rtrim(rS));
}
