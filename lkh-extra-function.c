// This is a wrapper function for the LKH main function, so that it can be called from the C++ code.
// It's supposed to be copy pasted into the LKHmain.c file (but that one is not versioned, hence this lone c file that's never called by itself).
int mainWrapper(std::vector<std::string> args
    // Convert the args string vector to a c style char array array) {
    int argc = args.size();
    if (argc <= 0)
        return 1337;
    std::vector<std::vector<char>> argv;
    for (unsigned int i=0; i<args.size(); ++i) {
        std::vector<char> str(args[i].begin(), args[i].end());
        str.push_back('\0');
        argv.push_back(str);
        // the original str will go out of scope and be 'deleted' now, but it's copied into the argv array, so should be ok.
    }
    return main(argc, &argv[0][0]);
}

// Of course, lets not forget the line to put in the header file
#include <vector>
#include <string>
int mainWrapper(std::vector<std::string> args);

// Ok, lots of tings changed - for now I only need to unclude the line "int mainWrapper(int argc, char* argv[]);" in the header file
// and rename the KLHmain.c main function to mainWrapper.

