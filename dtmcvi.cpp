#include "dtmcvi.h"

namespace acasx {

DTMCVI::DTMCVI(DTMC* dtmc): dtmc(dtmc)
{
    U = new double*[TIME_HORIZON+2];//U[T+1] is for the K-step expected cost when k>T (i.e. k>K), denoted JkBar
    int dtmcUStatesNum = dtmc->getUStatesNum();
    for(int i=0; i<TIME_HORIZON+2; ++i)
    {
        U[i] = new double[dtmcUStatesNum];
    }

    uStates=dtmc->getStates();
    //initialisation
    for (int i=0; i<dtmcUStatesNum;i++)
    {
        State_UnCtrl* s=uStates+i;
        if(s->getR()<=DTMCVI_COLLISION_R)
        {
            U[0][i]=1.0;
        }
        else
        {
            U[0][i]=0.0;
        }

    }

    map<int,double> TransitionStatesAndProbs;

    // repeat
    for(int iteration=1;iteration<=TIME_HORIZON;iteration++)
    {
        cout<<iteration<<endl;
        for (int i=0; i<dtmcUStatesNum;i++)
        {
            State_UnCtrl* s=uStates+i;
            assert (s->getOrder()==i);

            double prob=0;
            if(s->getR()>DTMCVI_COLLISION_R)
            {
                TransitionStatesAndProbs= dtmc->getTransitionStatesAndProbs(i);

                for (auto entry : TransitionStatesAndProbs)
                {
                    int nextStateOrder = entry.first;
                    prob += entry.second * U[iteration-1][nextStateOrder];
                }
            }
            U[iteration][i]=prob;

        }
    }

}

DTMCVI::~DTMCVI()
{
    for(int i=0; i<TIME_HORIZON+2; ++i)
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
    for(int k=0; k<=TIME_HORIZON;++k )
    {
        for (int su=0; su<dtmc->getUStatesNum();++su)
        {
            entryTimeDistributionFileWriter<<U[k][su]<<"\n";
        }
    }

    entryTimeDistributionFileWriter<<endl;
    entryTimeDistributionFileWriter.close();

}

}




