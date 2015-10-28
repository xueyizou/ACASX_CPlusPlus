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

#include "lookuptable.h"

#include <assert.h>

using namespace acasx;

LookupTable::LookupTable()
{
    string generatedFilesPrefix = "/home/xueyi/QtLib/ACASX/Debug/"; //"/home/xueyi/git/ACASX_3D/ACASX _3D/src/acasx3d/generation/generatedFiles/";

    cout<<"Reading look-up table...!"<<endl;

    std::chrono::time_point<std::chrono::system_clock> time0, time1;

    time0 = std::chrono::system_clock::now();

    numCStates = (2*MDP_H_NUM+1)*(2*MDP_OVY_NUM+1)*(2*MDP_IVY_NUM+1)*(MDP_RA_NUM);
    uint32_t numEntries1=numCStates*(TIME_HORIZON+2) + 1;

    numUStates = (DTMC_R_NUM+1)*(DTMC_RV_NUM+1)*(2*DTMC_THETA_NUM+1);
    uint32_t numEntries2=numUStates*(TIME_HORIZON+1);

    indexArr.reserve(numEntries1);
    costArr.reserve(numEntries1);
    actionArr.reserve(numEntries1);
    entryTimeDistributionArr.reserve(numEntries2);


    ifstream indexFileReader;
    indexFileReader.open(generatedFilesPrefix+"indexFile", ios_base::in);
    if(!indexFileReader)
    {
        cerr<<"failure in openning indexFile"<<endl;
        abort();
    }
    int index;
    while(indexFileReader>>index)
    {
        indexArr.push_back(index);

    }
    assert(indexArr.size()==numEntries1);
    indexFileReader.close();


    ifstream costFileReader;
    costFileReader.open(generatedFilesPrefix+"costFile", ios_base::in);
    if(!costFileReader)
    {
        cerr<<"failure in openning costFile"<<endl;
        abort();
    }
    double cost;
    while(costFileReader>>cost)
    {
        costArr.push_back(cost);
    }
    costFileReader.close();



    ifstream actionFileReader;
    actionFileReader.open(generatedFilesPrefix+"actionFile", ios_base::in);
    if(!actionFileReader)
    {
        cerr<<"failure in openning actionFile"<<endl;
        abort();
    }
    int action;
    while(actionFileReader>>action)
    {
        actionArr.push_back(action);

    }
    actionFileReader.close();



    ifstream entryTimeDistributionFileReader;
    entryTimeDistributionFileReader.open(generatedFilesPrefix+"entryTimeDistributionFile", ios_base::in);
    if(!entryTimeDistributionFileReader)
    {
        cerr<<"failure in openning entryTimeDistributionFile"<<endl;
        abort();
    }
    double entryTimeProb;
    while(entryTimeDistributionFileReader>>entryTimeProb)
    {
        entryTimeDistributionArr.push_back(entryTimeProb);

    }
    assert(entryTimeDistributionArr.size()==numEntries2);
    entryTimeDistributionFileReader.close();

    time1 = std::chrono::system_clock::now();
    cout<<"Done! The time for reading look-up table is "<< (static_cast<std::chrono::duration<double>>(time1-time0)).count() <<" senconds."<<endl;
}



LookupTable& LookupTable::getInstance()
{
   static LookupTable lookupTable;

    return lookupTable;
}
