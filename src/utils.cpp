#include "utils.h"

#include <algorithm>
#include <functional>
#include <cctype>
#include <locale>
#include <sstream>
#include <stdexcept>

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

//
// Check if string is an integer number
//
bool isInt(const string& s, bool allowNegative)
{
    string::const_iterator it = s.begin();
    if (allowNegative && *it == '-')
        ++it;
    while (it != s.end() && isdigit(*it))
        ++it;
    return !s.empty() && it == s.end();
}

//
// stoi and to_string workaround
//
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
int stoi(const string& str, size_t* pos, int base) {
    // String to int
    const char* begin = str.c_str() ;
    char* end = nullptr ;
    long value = std::strtol( begin, &end, base ) ;

    if( errno == ERANGE || value > std::numeric_limits<int>::max() )
        throw std::out_of_range( "stoi: out ofrange" ) ;
    if( end == str.c_str() )
        throw std::invalid_argument( "stoi: invalid argument" ) ;

    if(pos) *pos = end - begin ;

    return value ;
}

string to_string(int n) {
    // int to string
    std::ostringstream stm;
    stm << n;
    return stm.str();
}
#endif // defined
