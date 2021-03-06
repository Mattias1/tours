#include "utils.h"

#include <algorithm>
#include <functional>
#include <cctype>
#include <locale>
#include <sstream>
#include <stdexcept>

using namespace std;


//
// Copy int-vector references
//
vector<int> duplicate(const vector<int>& lst) {
    // Deep copy (which in this case is the same as a shallow copy)
    vector<int> result(lst.size());
    for (int i=0; i<lst.size(); ++i)
        result[i] = lst[i];
    return result;
}
vector<vector<int>> duplicate(const vector<vector<int>>& lst) {
    // Deep copy
    vector<vector<int>> result(lst.size());
    for (int i=0; i<lst.size(); ++i)
        result[i] = duplicate(lst[i]);
    return result;
}
vector<MatchingEdge> duplicate(const vector<MatchingEdge>& lst) {
    // Deep copy (which in this case is the same as a shallow copy)
    vector<MatchingEdge> result(lst.size());
    for (int i=0; i<lst.size(); ++i)
        result[i] = lst[i];
    return result;
}
vector<MatchingEdge*> duplicate(const vector<MatchingEdge*>& lst) {
    // Deep copy (which in this case is the same as a shallow copy)
    vector<MatchingEdge*> result(lst.size());
    for (int i=0; i<lst.size(); ++i)
        result[i] = lst[i];
    return result;
}
vector<vector<MatchingEdge*>> duplicate(const vector<vector<MatchingEdge*>>& lst) {
    // Deep copy
    vector<vector<MatchingEdge*>> result(lst.size());
    for (int i=0; i<lst.size(); ++i)
        result[i] = duplicate(lst[i]);
    return result;
}
vector<Edge*> duplicate(const vector<Edge*>& lst) {
    // Deep copy (which in this case is the same as a shallow copy)
    vector<Edge*> result(lst.size());
    for (int i=0; i<lst.size(); ++i)
        result[i] = lst[i];
    return result;
}

//
// Pointerize
//
vector<MatchingEdge*> pointerize(vector<MatchingEdge>& lst) {
    // Return a vector with pointers to the values in lst
    vector<MatchingEdge*> result;
    for (int i=0; i<lst.size(); ++i)
        result.push_back(&lst[i]);
    return result;
}

//
// Flatten a vector of int-vectors
//
vector<int> flatten(const vector<vector<int>>& lst) {
    vector<int> result; // Possible optimization: add in the correct size (the actual size in memory of the lst?)
    for (int i=0; i<lst.size(); ++i)
        for (int j=0; j<lst[i].size(); ++j)
            result.push_back(lst[i][j]);
    return result;
}
vector<MatchingEdge*> flatten(const vector<vector<MatchingEdge*>>& lst) {
    vector<MatchingEdge*> result; // Possible optimization: add in the correct size (the actual size in memory of the lst?)
    for (int i=0; i<lst.size(); ++i)
        for (int j=0; j<lst[i].size(); ++j)
            result.push_back(lst[i][j]);
    return result;
}

//
// Push back an entire list
//
void pushBackList(vector<Edge*>* pOriginalList, const vector<Edge*>& listToAdd) {
    for (int i=0; i<listToAdd.size(); ++i)
        pOriginalList->push_back(listToAdd[i]);
}
void pushBackList(vector<pair<int, int>>* pOriginalList, const vector<pair<int, int>>& listToAdd) {
    for (int i=0; i<listToAdd.size(); ++i)
        pOriginalList->push_back(listToAdd[i]);
}

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

vector<int> &splitInt(const string &s, char delim, vector<int> &elems) {
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        elems.push_back(stoi(item));
    }
    return elems;
}
vector<int> splitInt(const string& s, char delim) {
    vector<int> elems;
    splitInt(s, delim, elems);
    return elems;
}

//
// String join method
//
string join(const vector<int>& v, char delim) {
    stringstream ss;
    for(size_t i = 0; i < v.size(); ++i) {
        if(i != 0)
            ss << delim;
        ss << v[i];
    }
    return ss.str();
}
string join(const vector<MatchingEdge*>& v, char delim) {
    stringstream ss;
    for(size_t i = 0; i < v.size(); ++i) {
        if(i != 0)
            ss << delim;
        ss << v[i]->A;
        ss << delim;
        ss << v[i]->B;
        ss << delim;
        ss << v[i]->Demand;
    }
    return ss.str();
}
string join(const vector<MatchingEdge>& v, char delim) {
    stringstream ss;
    for(size_t i = 0; i < v.size(); ++i) {
        if(i != 0)
            ss << delim;
        ss << v[i].A;
        ss << delim;
        ss << v[i].B;
        ss << delim;
        ss << v[i].Demand;
    }
    return ss.str();
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
// Check if string is an integer (or double) number
//
bool isInt(const string& s, bool allowNegative /*=false*/)
{
    string::const_iterator it = s.begin();
    if (allowNegative && *it == '-')
        ++it;
    while (it != s.end() && isdigit(*it))
        ++it;
    return !s.empty() && it == s.end();
}

double isDouble(const string& s)
{
    istringstream iss(s);
    double d;
    char c;
    if (iss >> d && !(iss >> c));
        return d;
    return -1;
}


//
// Some debug helpers
//
string dbg(const string& s, int i) {
    // Output i times s
    string result = "";
    for (int z=0; z<i; ++z)
        result += s;
    return result;
}
string dbg(const vector<int>& v) {
    // Output the int vector
    string result = "[";
    for (int i=0; i<v.size(); ++i) {
        result += to_string(v[i]);
        if (i != v.size() - 1)
            result += ",";
    }
    return result + "]";
}
string dbg(const vector<vector<int>>& v) {
    // Output the vector of int vectors
    string result = "[";
    for (int i=0; i<v.size(); ++i) {
        for (int j=0; j<v[i].size(); ++j) {
            result += to_string(v[i][j]);
            if (j != v[i].size() - 1)
                result += ",";
        }
        if (i != v.size() - 1)
            result += " - ";
    }
    return result + "]";
}
string dbg(const vector<vector<vector<int>>>& v) {
    // Output the vector of vectors of int vectors
    string result = "[";
    for (int i=0; i<v.size(); ++i) {
        result += dbg(v[i]);
        if (i != v[i].size() - 1)
            result += ", \n";
    }
    return result + "]";
}
string dbg(const vector<MatchingEdge>& matchings) {
    // Output the matchings
    string result = "[";
    for (int i=0; i<matchings.size(); ++i) {
        result += to_string(matchings[i].A) + "-" + to_string(matchings[i].B) + ":" + to_string(matchings[i].Demand);
        if (i != matchings.size() - 1)
            result += ",";
    }
    return result + "]";
}
string dbg(const vector<vector<MatchingEdge>>& matchings) {
    // Output the vector of matchings
    string result = "[";
    for (int i=0; i<matchings.size(); ++i) {
        for (int j=0; j<matchings[i].size(); ++j) {
        result += to_string(matchings[i][j].A) + "-" + to_string(matchings[i][j].B) + ":" + to_string(matchings[i][j].Demand);
            if (j != matchings[i].size() - 1)
                result += ",";
        }
        if (i != matchings.size() - 1)
            result += " - ";
    }
    return result + "]";
}
string dbg(const vector<MatchingEdge*>& matchings) {
    // Output the matchings
    string result = "[";
    for (int i=0; i<matchings.size(); ++i) {
        result += to_string(matchings[i]->A) + "-" + to_string(matchings[i]->B) + ":" + to_string(matchings[i]->Demand);
        if (i != matchings.size() - 1)
            result += ",";
    }
    return result + "]";
}
string dbg(const vector<vector<MatchingEdge*>>& matchings) {
    // Output the vector of matchings
    string result = "[";
    for (int i=0; i<matchings.size(); ++i) {
        for (int j=0; j<matchings[i].size(); ++j) {
        result += to_string(matchings[i][j]->A) + "-" + to_string(matchings[i][j]->B) + ":" + to_string(matchings[i][j]->Demand);
            if (j != matchings[i].size() - 1)
                result += ",";
        }
        if (i != matchings.size() - 1)
            result += " - ";
    }
    return result + "]";
}
string dbg(const vector<vector<pair<int, int>>>& pairs) {
    // Output the vector of pairs
    string result = "[";
    for (int i=0; i<pairs.size(); ++i) {
        for (int j=0; j<pairs[i].size(); ++j) {
        result += "(" + to_string(pairs[i][j].first) + "," + to_string(pairs[i][j].second) + ")";
            if (j != pairs[i].size() - 1)
                result += ",";
        }
        if (i != pairs.size() - 1)
            result += " - ";
    }
    return result + "]";
}
string dbg(const vector<Edge*>& edges) {
    // Output the edges
    string result = "[";
    for (int i=0; i<edges.size(); ++i) {
        // Just to be sure...
        if (edges[i] == nullptr) {
            result += "null";
            continue;
        }
        if (edges[i]->pA == nullptr || edges[i]->pB == nullptr) {
            result += "null?-null?";
            continue;
        }
        // The original edge to string like it's supposed to be
        result += to_string(edges[i]->pA->vid) + "-" + to_string(edges[i]->pB->vid);
        if (i != edges.size() - 1)
            result += ",";
    }
    return result + "]";
}
string dbg(const vector<Edge*>* const pEdges) {
    // Output null or the edges
    if (pEdges == nullptr)
        return "null";
    return dbg(*pEdges);
}
string dbg(const vector<Vertex*>& vertices) {
    // Output the vertex ids
    string result = "[";
    for (int i=0; i<vertices.size(); ++i) {
        result += to_string(vertices[i]->vid);
        if (i != vertices.size() - 1)
            result += ",";
    }
    return result + "]";
}
string dbg(const vector<pair<int, list<int>>*> paths) {
    // Output the path (linked-)list
    string result = "";
    for (int p=0; p<paths.size(); ++p) {
        if (paths[p] == nullptr)
            continue;
        result += "  " + to_string(p) + ": ";
        for (auto iterVid = paths[p]->second.begin(); iterVid != paths[p]->second.end(); iterVid++) {
            if (iterVid != paths[p]->second.begin())
                result += ", ";
            result += to_string(*iterVid);
        }
        if (p != paths.size() - 1)
            result += "\n";
    }
    return result;
}
string dbg(const vector<bool> v) {
    // Output the boolean vector
    string result = "[";
    for (int i=0; i<v.size(); ++i) {
        result += v[i] ? "true" : "false";
        if (i != v.size() - 1)
            result += ",";
    }
    return result + "]";
}
string dbg(const vector<vector<bool>>& v) {
    // Output the vector of bool vectors
    string result = "[";
    for (int i=0; i<v.size(); ++i) {
        for (int j=0; j<v[i].size(); ++j) {
            result += v[i][j] ? "true" : "false";
            if (j != v[i].size() - 1)
                result += ",";
        }
        if (i != v.size() - 1)
            result += " - ";
    }
    return result + "]";
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
