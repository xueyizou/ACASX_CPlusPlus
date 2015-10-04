#include "dtmcvi.hpp"
#include "mdpvi.hpp"
#include "state_unctrl.hpp"
#include <map>
#include <iostream>
#include "dtmc.hpp"
#include <assert.h>
#include <fstream>

using namespace std;

namespace acasx {

const int DTMCVI::T=MDPVI::T;
const double DTMCVI::COLLISION_R=500;

DTMCVI::DTMCVI(DTMC* dtmc): dtmc(dtmc)
{
    U = new double*[T+2];//U[T+1] is for the K-step expected cost when k>T (i.e. k>K), denoted JkBar
    for(int i=0; i<T+2; ++i)
    {
        U[i] = new double[DTMC::numUStates];
    }

    uStates=dtmc->states();
    //initialisation
    for (int i=0; i<DTMC::numUStates;i++)
    {
        State_UnCtrl* s=uStates+i;
        if(s->getR()<=COLLISION_R)
        {
            U[0][i]=1.0;
        }
        else
        {
            U[0][i]=0.0;
        }

    }

    map<State_UnCtrl*,double> TransitionStatesAndProbs;

    // repeat
    for(int iteration=1;iteration<=T;iteration++)
    {
        cout<<iteration<<endl;
        for (int i=0; i<DTMC::numUStates;i++)
        {
            State_UnCtrl* s=uStates+i;
            assert (s->getOrder()==i);

            double prob=0;
            if(s->getR()>COLLISION_R)
            {
                TransitionStatesAndProbs= dtmc->getTransitionStatesAndProbs(s);

                for (auto entry : TransitionStatesAndProbs)
                {
                    State_UnCtrl* nextState = entry.first;
                    int nextStateOrder = nextState->getOrder();
                    prob += entry.second * U[iteration-1][nextStateOrder];
                }
            }
            U[iteration][i]=prob;

        }
    }

}

DTMCVI::~DTMCVI()
{
    for(int i=0; i<T+2; ++i)
    {
        delete[] U[i];
    }
    delete[] U;
}

void DTMCVI::storeValues()
{
    ofstream entryTimeDistributionFileWriter;
    entryTimeDistributionFileWriter.open("entryTimeDistributionFile", ios_base::out|ios_base::trunc);

    if(!entryTimeDistributionFileWriter.is_open())
    {
        cerr<<"failure in opening files"<<endl;
        return;
    }
    for(int k=0; k<=T;++k )
    {
        for (int su=0; su<DTMC::numUStates;++su)
        {
            entryTimeDistributionFileWriter<<U[k][su]<<"\n";
        }
    }

    entryTimeDistributionFileWriter<<endl;
    entryTimeDistributionFileWriter.close();

}

}




