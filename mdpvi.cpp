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

#include "mdpvi.h"

namespace acasx {

/**
 * Constructor.
 *
 * @param gamma discount &gamma; to be used.
 */
MDPVI::MDPVI(MDP *mdp): mdp(mdp)
{
    int numCStates = mdp->getCStatesNum();
    U = new double*[TIME_HORIZON+2];//U[T+1] is for the K-step expected cost when k>T (i.e. k>K), denoted JkBar
    for(int i=0; i<TIME_HORIZON+2; ++i)
    {
        U[i] = new double[numCStates];
    }

    cStates=mdp->getStates();
    //initialisation
    for (int i=0; i<numCStates;++i)
    {
        U[0][i]=mdp->reward(i, -1);
        U[TIME_HORIZON+1][i]=0;
    }

    double* JkBar = new double[numCStates];
    // repeat
    for(int iteration=1;iteration<=TIME_HORIZON; ++iteration)
    {
        cout<<iteration<<endl;
        for (int i=0; i<numCStates;++i)
        {
            vector<int> actions= mdp->actions(i);

            double aMax1 = -std::numeric_limits<double>::infinity();
            double aMax2 = -std::numeric_limits<double>::infinity();
            for (int& a : actions)
            {
                double aSum1=mdp->reward(i, a);
                double aSum2=mdp->reward(i, a);
                map<int,double> TransitionStatesAndProbs= mdp->getTransitionStatesAndProbs(i, a);

                for (auto& entry : TransitionStatesAndProbs)
                {
                    int nextStateOrder = entry.first;
                    aSum1 += entry.second * U[iteration-1][nextStateOrder];
                    aSum2 += entry.second * U[TIME_HORIZON+1][nextStateOrder];
                }

                if (aSum1 > aMax1)
                {
                    aMax1 = aSum1;
                }

                if (aSum2 > aMax2)
                {
                    aMax2 = aSum2;
                }

            }
            U[iteration][i]=aMax1;
            JkBar[i]=aMax2;
        }

        for(int i=0; i<numCStates;++i)
        {
            U[TIME_HORIZON+1][i]=JkBar[i];
        }

    }

    delete[] JkBar;
}

MDPVI::~MDPVI()
{
    for(int i=0; i<TIME_HORIZON+2; ++i)
    {
        delete[] U[i];
    }
    delete[] U;
}

//q-value: the expected accumulated reward from a state after committed to a certain action, and them acting optimally
double MDPVI::getQValue(int k, int stateOrder, int action)
{
    double QValue = 0;
    map<int,double> TransitionStatesAndProbs= mdp->getTransitionStatesAndProbs(stateOrder, action);

    for (auto& entry : TransitionStatesAndProbs)
    {
        assert(entry.second<=1);
        int nextStateOrder=entry.first;
        QValue += entry.second*  U[k-1][nextStateOrder];
    }
    return QValue;
}

// store the q-values in lookup tables
void MDPVI::storeQValues()
{
    ofstream indexFileWriter;
    indexFileWriter.open("indexFile", ios_base::out|ios_base::trunc);
    ofstream costFileWriter;
    costFileWriter.open("costFile", ios_base::out|ios_base::trunc);
    ofstream actionFileWriter;
    actionFileWriter.open("actionFile", ios_base::out|ios_base::trunc);

    if(!indexFileWriter.is_open() || !costFileWriter.is_open() || !actionFileWriter.is_open())
    {
        cerr<<"failure in opening files"<<endl;
        abort();
    }

    int index=0;
    for (int i=0; i<mdp->getCStatesNum(); ++i)//T=k=0
    {
        indexFileWriter<<index<<"\n";
        actionFileWriter<<0<<"\n";  //leaving states, using O to avoid arrayIndexOutofRange problem
        index++;
        costFileWriter<<U[0][i]<<"\n";
    }

    for(int k=1; k<=TIME_HORIZON+1; ++k )
    {
        for (int i=0; i<mdp->getCStatesNum();++i)
        {
            vector<int> actions=mdp->actions(i);
            indexFileWriter<<index<<"\n";
            for (int& a :actions )
            {
                actionFileWriter<<a<<"\n";
                double QValue = getQValue(k, i, a);
                costFileWriter<<QValue<<"\n";
            }
            index+=actions.size();
        }
    }

    indexFileWriter<<index<<endl;
    actionFileWriter<<endl;
    costFileWriter<<endl;

    indexFileWriter.close();
    costFileWriter.close();
    actionFileWriter.close();

}


}

