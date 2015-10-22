#ifndef LOOKUPTABLE_H
#define LOOKUPTABLE_H

#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>

#include "constants.h"

using namespace std;
using namespace acasx;

class LookupTable
{
public:
    static LookupTable& getInstance();

    int numCStates;
    int numUStates;
    vector<int> indexArr;
    vector<double> costArr;
    vector<int> actionArr;
    vector<double> entryTimeDistributionArr;

private:
    LookupTable();
    static string generatedFilesPrefix;

};

#endif // LOOKUPTABLE_H

