#include "mdp.hpp"
#include <cmath>
#include "state_ctrl.hpp"
#include <map>
#include "utils.hpp"
#include <chrono>
#include <ctime>


using namespace std;


namespace acasx {

const double MDP::UPPER_H=600.0;
const double MDP::UPPER_VY=70.0;

const int MDP::nh = 10;//10
const int MDP::noVy=7;//7
const int MDP::niVy= 7;//7
const int MDP::nra=7;

const double MDP::hRes = UPPER_H/nh;
const double MDP::oVRes = UPPER_VY/noVy;
const double MDP::iVRes = UPPER_VY/niVy;

const double MDP::WHITE_NOISE_SDEV=3.0;

const int MDP::numCStates= (2*nh+1)*(2*noVy+1)*(2*niVy+1)*nra;

MDP::MDP()
{
//    totalStates.resize(numCStates);
    cStates = new State_Ctrl[numCStates];
    cout<<cStates[5].getH()<<endl;
    if(!cStates)
    {
        cout<<cStates<<endl;
        cerr<<"failure in dynamic memory allocation "<<endl;
        return;
    }

    for(int hIdx=-nh; hIdx<=nh;hIdx++)//
    {
        for(int oVyIdx=-noVy; oVyIdx<=noVy;oVyIdx++)//
        {
            for(int iVyIdx=-niVy; iVyIdx<=niVy;iVyIdx++)
            {
                for(int raIdx=0; raIdx<nra;raIdx++)//
                {
                    State_Ctrl* ptr = new State_Ctrl (hIdx, oVyIdx, iVyIdx, raIdx);
                    cStates[State_Ctrl::calOrder(hIdx, oVyIdx, iVyIdx, raIdx)] =*ptr;
//                    totalStates[State_Ctrl::calOrder(hIdx, oVyIdx, iVyIdx, raIdx)] =*ptr;
                    delete ptr;
                }
            }
        }
    }
    cout<<cStates[5].getH()<<endl;

    sigmaPointsA.push_back(make_tuple(0.0,0.0,1.0/2));
    sigmaPointsA.push_back(make_tuple(0.0,sqrt(2.0)*WHITE_NOISE_SDEV,1.0/4));
    sigmaPointsA.push_back(make_tuple(0.0,-sqrt(2.0)*WHITE_NOISE_SDEV,1.0/4));

    sigmaPointsB.push_back(make_tuple(0.0,0.0,1.0/3));
    sigmaPointsB.push_back(make_tuple(sqrt(3.0)*WHITE_NOISE_SDEV,0.0,1.0/6));
    sigmaPointsB.push_back(make_tuple(-sqrt(3.0)*WHITE_NOISE_SDEV,0.0,1.0/6));
    sigmaPointsB.push_back(make_tuple(0.0,sqrt(3.0)*WHITE_NOISE_SDEV,1.0/6));
    sigmaPointsB.push_back(make_tuple(0.0,-sqrt(3.0)*WHITE_NOISE_SDEV,1.0/6));

}


MDP::~MDP()
{
    delete[] cStates;
}

State_Ctrl* MDP::states(void){
    return cStates;
}

/**
 * Get the set of actions for state s.
 *
 * @param s the state.
 * @return the list of actions for state s.
 */
vector<int> MDP::actions(State_Ctrl* cstate)
{
    vector<int> actions;

    if(cstate->getH()==UPPER_H || cstate->getoVy()== UPPER_VY)
    {
        actions.push_back(0);//"COC"
        actions.push_back(4);//"SDES25"
        return actions;
    }
    if(cstate->getH()==-UPPER_H || cstate->getoVy()== -UPPER_VY)
    {
        actions.push_back(0);//"COC"
        actions.push_back(3);//"SCL25"
        return actions;
    }
    else if(cstate->getRa()==0)//COC
    {
        actions.push_back(0);//"COC"
        actions.push_back(1);//"CL25"
        actions.push_back(2);//"DES25"
        return actions;
    }
    else if(cstate->getRa()==1)//CL25
    {
        actions.push_back(0);//"COC"
        actions.push_back(1);//"CL25"
        actions.push_back(4);//"SDES25"
        actions.push_back(5);//"SCL42"
        return actions;
    }
    else if(cstate->getRa()==2)//DES25
    {
        actions.push_back(0);//"COC"
        actions.push_back(2);//"DES25"
        actions.push_back(3);//"SCL25"
        actions.push_back(6);//"SDES42"
        return actions;
    }
    else if(cstate->getRa()==3)//"SCL25"
    {
        actions.push_back(0);//"COC"
        actions.push_back(3);//"SCL25"
        actions.push_back(4);//"SDES25"
        actions.push_back(5);//"SCL42"
        return actions;
    }
    else if(cstate->getRa()==4)//"SDES25"
    {
        actions.push_back(0);//"COC"
        actions.push_back(3);//"SCL25"
        actions.push_back(4);//"SDES25"
        actions.push_back(6);//"SDES42"
        return actions;
    }
    else if(cstate->getRa()==5)//"SCL42"
    {
        actions.push_back(0);//"COC"
        actions.push_back(3);//"SCL25"
        actions.push_back(4);//"SDES25"
        actions.push_back(5);//"SCL42"
        return actions;
    }
    else if(cstate->getRa()==6)//"SDES42"
    {
        actions.push_back(0);//"COC"
        actions.push_back(3);//"SCL25"
        actions.push_back(4);//"SDES25"
        actions.push_back(6);//"SDES42"
        return actions;
    }
    else
    {
        cerr<<"Something wrong happend in ACASXMDP.actions(State s)."<<endl;
        return actions;
    }

}

map<State_Ctrl*,double> MDP::getTransitionStatesAndProbs(State_Ctrl* cstate, int actionCode)
{
//    std::chrono::time_point<std::chrono::system_clock> time0, time1;
//    time0 = std::chrono::system_clock::now();

    map<State_Ctrl*,double> TransitionStatesMapProbs;

    double targetV=getActionV(actionCode);
    double accel=getActionA(actionCode);
    vector< pair<State_Ctrl*,double> > nextStateandProbabilities;

    if( (accel>0 && targetV>cstate->getoVy() && cstate->getoVy()<UPPER_VY)
            || (accel<0 && targetV<cstate->getoVy() && cstate->getoVy()>-UPPER_VY))
    {// own aircraft follows a RA other than COC

        for(auto iter=sigmaPointsA.begin(); iter!=sigmaPointsA.end(); ++iter)
        {
            double oAy=accel;
            double iAy=get<1>(*iter);
            double sigmaP=get<2>(*iter);

            double hP= cstate->getH()+ (cstate->getiVy()-cstate->getoVy()) + 0.5*(iAy-oAy);
            double oVyP= max(- UPPER_VY, min( UPPER_VY, cstate->getoVy()+oAy));
            double iVyP= max(- UPPER_VY, min( UPPER_VY, cstate->getiVy()+iAy));
            int raP=actionCode;

            int hIdxL = (int)floor(hP/hRes);
            int oVyIdxL = (int)floor(oVyP/oVRes);
            int iVyIdxL = (int)floor(iVyP/iVRes);
            for(int i=0;i<=1;i++)
            {
                int hIdx = (i==0? hIdxL : hIdxL+1);
                int hIdxP= hIdx< -nh? -nh: (hIdx>nh? nh : hIdx);
                for(int j=0;j<=1;j++)
                {
                    int oVzIdx = (j==0? oVyIdxL : oVyIdxL+1);
                    int oVzIdxP= oVzIdx<-noVy? -noVy: (oVzIdx>noVy? noVy : oVzIdx);
                    for(int k=0;k<=1;k++)
                    {
                        int iVzIdx = (k==0? iVyIdxL : iVyIdxL+1);
                        int iVzIdxP= iVzIdx<-niVy? -niVy: (iVzIdx>niVy? niVy : iVzIdx);

                        State_Ctrl* nextState=(cStates+State_Ctrl::calOrder(hIdxP, oVzIdxP, iVzIdxP, raP));
                        double probability= sigmaP*(1-abs(hIdx-hP/hRes))*(1-abs(oVzIdx-oVyP/oVRes))*(1-abs(iVzIdx-iVyP/iVRes));
                        nextStateandProbabilities.push_back(make_pair(nextState,probability) );
                    }
                }
            }

        }

    }
    else
    {
        for(auto iter=sigmaPointsB.begin(); iter!=sigmaPointsB.end(); ++iter)
        {
            double oAy=get<0>(*iter);
            double iAy=get<1>(*iter);
            double sigmaP=get<2>(*iter);

            double hP= cstate->getH()+ (cstate->getiVy()-cstate->getoVy()) + 0.5*(iAy-oAy);
            double oVyP= max(-UPPER_VY, min(UPPER_VY, cstate->getoVy()+oAy));
            double iVyP= max(-UPPER_VY, min(UPPER_VY, cstate->getiVy()+iAy));
            int raP=actionCode;

            int hIdxL = (int)floor(hP/hRes);
            int oVyIdxL = (int)floor(oVyP/oVRes);
            int iVyIdxL = (int)floor(iVyP/iVRes);
            for(int i=0;i<=1;i++)
            {
                int hIdx = (i==0? hIdxL : hIdxL+1);
                int hIdxP= hIdx< -nh? -nh: (hIdx>nh? nh : hIdx);
                for(int j=0;j<=1;j++)
                {
                    int oVyIdx = (j==0? oVyIdxL : oVyIdxL+1);
                    int oVyIdxP= oVyIdx<-noVy? -noVy: (oVyIdx>noVy? noVy : oVyIdx);
                    for(int k=0;k<=1;k++)
                    {
                        int iVyIdx = (k==0? iVyIdxL : iVyIdxL+1);
                        int iVyIdxP= iVyIdx<-niVy? -niVy: (iVyIdx>niVy? niVy : iVyIdx);

                        State_Ctrl* nextState= (cStates+State_Ctrl::calOrder(hIdxP, oVyIdxP, iVyIdxP,raP));
                        double probability= sigmaP*(1-abs(hIdx-hP/hRes))*(1-abs(oVyIdx-oVyP/oVRes))*(1-abs(iVyIdx-iVyP/iVRes));
                        nextStateandProbabilities.push_back(make_pair(nextState,probability) );
                    }
                }
            }

        }

    }
//    time1 = std::chrono::system_clock::now();
//    cout<<"map time: "<< (static_cast<std::chrono::duration<double>>(time1-time0)).count() <<" senconds."<<endl;

    for(auto iter=nextStateandProbabilities.begin(); iter != nextStateandProbabilities.end(); ++iter)
    {
        State_Ctrl* nextState=iter->first;
        auto iPairFound = TransitionStatesMapProbs.find(nextState);
        if(iPairFound != TransitionStatesMapProbs.end())
        {
            TransitionStatesMapProbs[nextState] += iter->second;
        }
        else
        {
            TransitionStatesMapProbs.insert(make_pair(nextState, iter->second));
        }

    }

//    time1 = std::chrono::system_clock::now();
//    cout<<"map time: "<< (static_cast<std::chrono::duration<double>>(time1-time0)).count() <<" senconds."<<endl;

    return TransitionStatesMapProbs;
}


double MDP::reward(State_Ctrl* cstate,int actionCode)
{
    if(actionCode==-1)//terminate
    {
        if(abs(cstate->getH())<100)
        {//NMAC
            return -10000;
        }
        else
        {
            return 0;
        }
    }

    if(actionCode==1)
    {
        if(cstate->getoVy()>0)
        {
            return -50;
        }
        else if(cstate->getoVy()<0)
        {
            return -100;
        }

    }
    if(actionCode==2)
    {
        if(cstate->getoVy()>0)
        {
            return -100;
        }
        else if(cstate->getoVy()<0)
        {
            return -50;
        }
    }

    if(actionCode==0)//"COC"
    {//clear of conflict
        return 100;
    }

    if( ((cstate->getRa()==1||cstate->getRa()==3) && actionCode==5)
            || ((cstate->getRa()==2||cstate->getRa()==4)&& actionCode==6) )
    {//strengthening
        return -500;
    }

    if( ((cstate->getRa()==1 || cstate->getRa()==3 || cstate->getRa()==5)&& actionCode==4)
            || ((cstate->getRa()==2 || cstate->getRa()==4 || cstate->getRa()==6)&& actionCode==3) )
    {//reversal
        return -1000;
    }

    return 0;
}


}
