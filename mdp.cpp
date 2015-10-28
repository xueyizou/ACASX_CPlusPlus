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

#include "mdp.h"


namespace acasx {

// sigma points for sampling
vector< tuple<double, double, double> > sigmaPointsA;
vector< tuple<double, double, double> > sigmaPointsB;

MDP::MDP()
{
    numCStates= (2*MDP_H_NUM+1)*(2*MDP_OVY_NUM+1)*(2*MDP_IVY_NUM+1)*MDP_RA_NUM;
    cStates = new State_Ctrl[numCStates];

    if(!cStates)
    {
        cout<<cStates<<endl;
        cerr<<"failure in dynamic memory allocation "<<endl;
        abort();
    }

    for(int hIdx=-MDP_H_NUM; hIdx<=MDP_H_NUM; ++hIdx)//
    {
        for(int oVyIdx=-MDP_OVY_NUM; oVyIdx<=MDP_OVY_NUM; ++oVyIdx)//
        {
            for(int iVyIdx=-MDP_IVY_NUM; iVyIdx<=MDP_IVY_NUM; ++iVyIdx)
            {
                for(int raIdx=0; raIdx<MDP_RA_NUM; ++raIdx)//
                {
                    State_Ctrl* ptr = new State_Ctrl (hIdx, oVyIdx, iVyIdx, raIdx);
                    cStates[State_Ctrl::calOrder(hIdx, oVyIdx, iVyIdx, raIdx)] =*ptr;
                    delete ptr;
                }
            }
        }
    }

    sigmaPointsA.push_back(make_tuple(0.0,0.0,1.0/2));
    sigmaPointsA.push_back(make_tuple(0.0,sqrt(2.0)*MDP_WHITE_NOISE_SDEV,1.0/4));
    sigmaPointsA.push_back(make_tuple(0.0,-sqrt(2.0)*MDP_WHITE_NOISE_SDEV,1.0/4));

    sigmaPointsB.push_back(make_tuple(0.0,0.0,1.0/3));
    sigmaPointsB.push_back(make_tuple(sqrt(3.0)*MDP_WHITE_NOISE_SDEV,0.0,1.0/6));
    sigmaPointsB.push_back(make_tuple(-sqrt(3.0)*MDP_WHITE_NOISE_SDEV,0.0,1.0/6));
    sigmaPointsB.push_back(make_tuple(0.0,sqrt(3.0)*MDP_WHITE_NOISE_SDEV,1.0/6));
    sigmaPointsB.push_back(make_tuple(0.0,-sqrt(3.0)*MDP_WHITE_NOISE_SDEV,1.0/6));

}


MDP::~MDP()
{
    delete[] cStates;
}



/**
 * Get the set of actions for state s.
 *
 * @param s the state.
 * @return the list of actions for state s.
 */
vector<int> MDP::actions(int cStateOrder)
{
    vector<int> actions;

    State_Ctrl* cStatePtr = cStates+cStateOrder;

    if(cStatePtr->getH()==MDP_UPPER_H || cStatePtr->getoVy()== MDP_UPPER_VY)
    {
        actions.push_back(0);//"COC"
        actions.push_back(4);//"SDES25"
        return actions;
    }
    if(cStatePtr->getH()==-MDP_UPPER_H || cStatePtr->getoVy()== -MDP_UPPER_VY)
    {
        actions.push_back(0);//"COC"
        actions.push_back(3);//"SCL25"
        return actions;
    }
    else if(cStatePtr->getRa()==0)//COC
    {
        actions.push_back(0);//"COC"
        actions.push_back(1);//"CL25"
        actions.push_back(2);//"DES25"
        return actions;
    }
    else if(cStatePtr->getRa()==1)//CL25
    {
        actions.push_back(0);//"COC"
        actions.push_back(1);//"CL25"
        actions.push_back(4);//"SDES25"
        actions.push_back(5);//"SCL42"
        return actions;
    }
    else if(cStatePtr->getRa()==2)//DES25
    {
        actions.push_back(0);//"COC"
        actions.push_back(2);//"DES25"
        actions.push_back(3);//"SCL25"
        actions.push_back(6);//"SDES42"
        return actions;
    }
    else if(cStatePtr->getRa()==3)//"SCL25"
    {
        actions.push_back(0);//"COC"
        actions.push_back(3);//"SCL25"
        actions.push_back(4);//"SDES25"
        actions.push_back(5);//"SCL42"
        return actions;
    }
    else if(cStatePtr->getRa()==4)//"SDES25"
    {
        actions.push_back(0);//"COC"
        actions.push_back(3);//"SCL25"
        actions.push_back(4);//"SDES25"
        actions.push_back(6);//"SDES42"
        return actions;
    }
    else if(cStatePtr->getRa()==5)//"SCL42"
    {
        actions.push_back(0);//"COC"
        actions.push_back(3);//"SCL25"
        actions.push_back(4);//"SDES25"
        actions.push_back(5);//"SCL42"
        return actions;
    }
    else if(cStatePtr->getRa()==6)//"SDES42"
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

//describe the transitions, return a map of next states to probabilities from a given state.
map<int,double> MDP::getTransitionStatesAndProbs(int cStateOrder, int actionCode)
{
//    std::chrono::time_point<std::chrono::system_clock> time0, time1;
//    time0 = std::chrono::system_clock::now();

    State_Ctrl* cStatePtr = cStates+cStateOrder;

    map<int,double> TransitionStatesMapProbs;

    double targetV=getActionV(actionCode);
    double accel=getActionA(actionCode);
    vector< pair<int,double> > nextStateandProbabilities;

    if( (accel>0 && targetV>cStatePtr->getoVy() && cStatePtr->getoVy()<MDP_UPPER_VY)
            || (accel<0 && targetV<cStatePtr->getoVy() && cStatePtr->getoVy()>-MDP_UPPER_VY))
    {// own aircraft follows a RA other than COC

        for(auto iter=sigmaPointsA.begin(); iter!=sigmaPointsA.end(); ++iter)
        {
            double oAy=accel;
            double iAy=get<1>(*iter);
            double sigmaP=get<2>(*iter);

            double hP= cStatePtr->getH()+ (cStatePtr->getiVy()-cStatePtr->getoVy()) + 0.5*(iAy-oAy);
            double oVyP= max(- MDP_UPPER_VY, min( MDP_UPPER_VY, cStatePtr->getoVy()+oAy));
            double iVyP= max(- MDP_UPPER_VY, min( MDP_UPPER_VY, cStatePtr->getiVy()+iAy));
            int raP=actionCode;

             // the following uses the method of linear interpolation for approximating the states
            int hIdxL = (int)floor(hP/MDP_H_RES);
            int oVyIdxL = (int)floor(oVyP/MDP_OV_RES);
            int iVyIdxL = (int)floor(iVyP/MDP_IV_RES);
            for(int i=0;i<=1;i++)
            {
                int hIdx = (i==0? hIdxL : hIdxL+1);
                int hIdxP= hIdx< -MDP_H_NUM? -MDP_H_NUM: (hIdx>MDP_H_NUM? MDP_H_NUM : hIdx);
                for(int j=0;j<=1;j++)
                {
                    int oVzIdx = (j==0? oVyIdxL : oVyIdxL+1);
                    int oVzIdxP= oVzIdx<-MDP_OVY_NUM? -MDP_OVY_NUM: (oVzIdx>MDP_OVY_NUM? MDP_OVY_NUM : oVzIdx);
                    for(int k=0;k<=1;k++)
                    {
                        int iVzIdx = (k==0? iVyIdxL : iVyIdxL+1);
                        int iVzIdxP= iVzIdx<-MDP_IVY_NUM? -MDP_IVY_NUM: (iVzIdx>MDP_IVY_NUM? MDP_IVY_NUM : iVzIdx);

                        int nextStateOrder= State_Ctrl::calOrder(hIdxP, oVzIdxP, iVzIdxP, raP);
                        double probability= sigmaP*(1-fabs(hIdx-hP/MDP_H_RES))*(1-fabs(oVzIdx-oVyP/MDP_OV_RES))*(1-fabs(iVzIdx-iVyP/MDP_IV_RES));
                        nextStateandProbabilities.push_back(make_pair(nextStateOrder,probability) );
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

            double hP= cStatePtr->getH()+ (cStatePtr->getiVy()-cStatePtr->getoVy()) + 0.5*(iAy-oAy);
            double oVyP= max(-MDP_UPPER_VY, min(MDP_UPPER_VY, cStatePtr->getoVy()+oAy));
            double iVyP= max(-MDP_UPPER_VY, min(MDP_UPPER_VY, cStatePtr->getiVy()+iAy));
            int raP=actionCode;

             // the following uses the method of linear interpolation for approximating the states
            int hIdxL = (int)floor(hP/MDP_H_RES);
            int oVyIdxL = (int)floor(oVyP/MDP_OV_RES);
            int iVyIdxL = (int)floor(iVyP/MDP_IV_RES);
            for(int i=0;i<=1;i++)
            {
                int hIdx = (i==0? hIdxL : hIdxL+1);
                int hIdxP= hIdx< -MDP_H_NUM? -MDP_H_NUM: (hIdx>MDP_H_NUM? MDP_H_NUM : hIdx);
                for(int j=0;j<=1;j++)
                {
                    int oVyIdx = (j==0? oVyIdxL : oVyIdxL+1);
                    int oVyIdxP= oVyIdx<-MDP_OVY_NUM? -MDP_OVY_NUM: (oVyIdx>MDP_OVY_NUM? MDP_OVY_NUM : oVyIdx);
                    for(int k=0;k<=1;k++)
                    {
                        int iVyIdx = (k==0? iVyIdxL : iVyIdxL+1);
                        int iVyIdxP= iVyIdx<-MDP_IVY_NUM? -MDP_IVY_NUM: (iVyIdx>MDP_IVY_NUM? MDP_IVY_NUM : iVyIdx);

                        int nextStateOrder= State_Ctrl::calOrder(hIdxP, oVyIdxP, iVyIdxP,raP);
                        double probability= sigmaP*(1-fabs(hIdx-hP/MDP_H_RES))*(1-fabs(oVyIdx-oVyP/MDP_OV_RES))*(1-fabs(iVyIdx-iVyP/MDP_IV_RES));
                        nextStateandProbabilities.push_back(make_pair(nextStateOrder,probability) );
                    }
                }
            }

        }

    }
//    time1 = std::chrono::system_clock::now();
//    cout<<"map time: "<< (static_cast<std::chrono::duration<double>>(time1-time0)).count() <<" senconds."<<endl;

    //merge entries with the same keys by summing up their values
    for(auto entry:nextStateandProbabilities)
    {
        int nextStateOrder=entry.first;
        auto iPairFound = TransitionStatesMapProbs.find(nextStateOrder);
        if(iPairFound != TransitionStatesMapProbs.end())
        {
            TransitionStatesMapProbs[nextStateOrder] += entry.second;
        }
        else
        {
            TransitionStatesMapProbs.insert(entry);
        }

    }

//    time1 = std::chrono::system_clock::now();
//    cout<<"map time: "<< (static_cast<std::chrono::duration<double>>(time1-time0)).count() <<" senconds."<<endl;

    return TransitionStatesMapProbs;
}


// return the reward from a state by doing an action
double MDP::reward(int cStateOrder, int actionCode)
{
    State_Ctrl* cStatePtr = cStates+cStateOrder;

    if(actionCode==-1)//terminate
    {
        if(fabs(cStatePtr->getH())<100)
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
        if(cStatePtr->getoVy()>0)
        {
            return -50;
        }
        else if(cStatePtr->getoVy()<0)
        {
            return -100;
        }

    }
    if(actionCode==2)
    {
        if(cStatePtr->getoVy()>0)
        {
            return -100;
        }
        else if(cStatePtr->getoVy()<0)
        {
            return -50;
        }
    }

    if(actionCode==0)//"COC"
    {//clear of conflict
        return 100;
    }

    if( ((cStatePtr->getRa()==1||cStatePtr->getRa()==3) && actionCode==5)
            || ((cStatePtr->getRa()==2||cStatePtr->getRa()==4)&& actionCode==6) )
    {//strengthening
        return -500;
    }

    if( ((cStatePtr->getRa()==1 || cStatePtr->getRa()==3 || cStatePtr->getRa()==5)&& actionCode==4)
            || ((cStatePtr->getRa()==2 || cStatePtr->getRa()==4 || cStatePtr->getRa()==6)&& actionCode==3) )
    {//reversal
        return -1000;
    }

    return 0;
}


}
