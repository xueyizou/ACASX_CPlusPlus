#include "mdpvi.hpp"
#include <map>
#include "mdp.hpp"
#include "state_ctrl.hpp"
#include <limits>
#include <fstream>


using namespace std;

namespace acasx {

const int MDPVI::T=20;

/**
 * Constructor.
 *
 * @param gamma discount &gamma; to be used.
 */
MDPVI::MDPVI(MDP *mdp): mdp(mdp)
{
    U = new double*[T+2];//U[T+1] is for the K-step expected cost when k>T (i.e. k>K), denoted JkBar
    for(int i=0; i<T+2; ++i)
    {
        U[i] = new double[MDP::numCStates];
    }

    states=mdp->states();
    //initialisation
    for (int i=0; i<MDP::numCStates;++i)
    {
        State_Ctrl* s=states+i;
        U[0][i]=mdp->reward(s, -1);
        U[T+1][i]=0;
    }

    double* JkBar = new double[MDP::numCStates];
    // repeat
    for(int iteration=1;iteration<=T;++iteration)
    {
        cout<<iteration<<endl;
        for (int i=0; i<MDP::numCStates;++i)
        {
            State_Ctrl* s=states+i;
            if(s->getOrder()!=i)
            {
                cerr<<"error happens in valueIteration() + s.getOrder()!=i"<<endl;
            }
            vector<int> actions= mdp->actions(s);

            double aMax1 = -std::numeric_limits<double>::infinity();
            double aMax2 = -std::numeric_limits<double>::infinity();
            for (int& a : actions)
            {
                double aSum1=mdp->reward(s, a);
                double aSum2=mdp->reward(s, a);
                map<State_Ctrl*,double> TransitionStatesAndProbs= mdp->getTransitionStatesAndProbs(s, a);

                for (auto& entry : TransitionStatesAndProbs)
                {
                    State_Ctrl* nextState = entry.first;
                    int nextStateOrder = nextState->getOrder();
                    aSum1 += entry.second * U[iteration-1][nextStateOrder];
                    aSum2 += entry.second * U[T+1][nextStateOrder];
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

        for(int i=0; i<MDP::numCStates;++i)
        {
            U[T+1][i]=JkBar[i];
        }

    }

    delete[] JkBar;
}

MDPVI::~MDPVI()
{
    for(int i=0; i<T+2; ++i)
    {
        delete[] U[i];
    }
    delete[] U;
}

double MDPVI::getQValue(int k,State_Ctrl* state, int action)
{
    double QValue = 0;
    map<State_Ctrl*,double> TransitionStatesAndProbs= mdp->getTransitionStatesAndProbs(state, action);

    for (auto& entry : TransitionStatesAndProbs)
    {
        if(entry.second>1)
        {
            cerr<<entry.second<<" greater than 1"<<endl;
        }
        State_Ctrl* nextState = entry.first;
        int nextStateOrder=nextState->getOrder();
        QValue += entry.second*  U[k-1][nextStateOrder];
    }
    return QValue;
}

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
        return;
    }

    int index=0;
    for (int i=0; i<MDP::numCStates;i++)//T=k=0
    {
        indexFileWriter<<index<<"\n";
        actionFileWriter<<0<<"\n";  //leaving states, using O to avoid arrayIndexOutofRange problem
        index++;
        costFileWriter<<U[0][i]<<"\n";
    }

    for(int k=1; k<=T+1;k++ )
    {
        for (int i=0; i<MDP::numCStates;i++)
        {
            State_Ctrl* s=states+i;
            vector<int> actions=mdp->actions(s);
            indexFileWriter<<index<<"\n";
            for (int& a :actions )
            {
                actionFileWriter<<a<<"\n";
                double QValue = getQValue(k, s, a);
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

