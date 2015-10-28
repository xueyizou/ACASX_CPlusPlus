/* *************************************************************************************
 * Copyright (C) Xueyi Zou - All Rights Reserved
 * Written by Xueyi Zou <xz972@york.ac.uk>, 2015
 * You are free to use/modify/distribute this file for whatever purpose!
 -----------------------------------------------------------------------
 |THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
 |WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
 |AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
 |DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
 |MISUSING THIS SOFTWARE.
 ------------------------------------------------------------------------

 **************************************************************************************/

#ifndef LOOKUPTABLE_H
#define LOOKUPTABLE_H


//for loading the lookup tables

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
    static LookupTable& getInstance();//singleton to load the lookup tables only once.

    int numCStates;//number of controlled states
    int numUStates;//number of uncontrolled states
    vector<int> indexArr; // storing the indice
    vector<double> costArr;// storing the costs
    vector<int> actionArr;// storing the actions
    vector<double> entryTimeDistributionArr;//storing the entry Time Distribution

private:
    LookupTable();
    static string generatedFilesPrefix;

};

#endif // LOOKUPTABLE_H

