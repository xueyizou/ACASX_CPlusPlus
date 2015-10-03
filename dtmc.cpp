#include "dtmc.hpp"
#include "state_unctrl.hpp"
#include <iostream>
#include <cmath>
#include "double2d.hpp"

using namespace std;

namespace acasx {

const double DTMC::UPPER_R=12200;//20000
const double DTMC::UPPER_RV=610;//1000
const double DTMC::UPPER_THETA=180.0;

const int DTMC::nr = 61;//61
const int DTMC::nrv= 61;//61
const int DTMC::ntheta= 36;//36

const double DTMC::rRes = UPPER_R/nr;
const double DTMC::rvRes = UPPER_RV/nrv;
const double DTMC::thetaRes = UPPER_THETA/ntheta;

const double DTMC::WHITE_NOISE_SDEV=3.0;
const double DTMC::WHITE_NOISE_SDEV_Angle=2.0;

const int DTMC::numUStates= (nr+1)*(nrv+1)*(2*ntheta+1);

DTMC::DTMC()
{
    uStates = new State_UnCtrl[numUStates];
    if(!uStates)
    {
        cout<<uStates<<endl;
        cerr<<"failure in dynamic memory allocation "<<endl;
        return;
    }

    for(int rIdx=0; rIdx<=nr;rIdx++)//
    {
        for(int rvIdx=0; rvIdx<=nrv;rvIdx++)//
        {
            for(int thetaIdx=-ntheta; thetaIdx<=ntheta;thetaIdx++)
            {
                State_UnCtrl* ptr = new State_UnCtrl(rIdx, rvIdx, thetaIdx);
                uStates[State_UnCtrl::calOrder(rIdx, rvIdx, thetaIdx)]=*ptr;
                delete ptr;
            }
        }
    }

    sigmaPoints1.push_back(make_tuple(0.0,0.0,1.0/3));
    sigmaPoints1.push_back(make_tuple(sqrt(3.0)*2*WHITE_NOISE_SDEV,0.0,1.0/6));
    sigmaPoints1.push_back(make_tuple(-sqrt(3.0)*2*WHITE_NOISE_SDEV,0.0,1.0/6));
    sigmaPoints1.push_back(make_tuple(0.0,sqrt(3.0)*2*WHITE_NOISE_SDEV,1.0/6));
    sigmaPoints1.push_back(make_tuple(0.0,-sqrt(3.0)*2*WHITE_NOISE_SDEV,1.0/6));

    sigmaPoints2.push_back(make_tuple(0.0,0.0,1.0/3));
    sigmaPoints2.push_back(make_tuple(sqrt(3.0)*2*WHITE_NOISE_SDEV,0.0,1.0/6));
    sigmaPoints2.push_back(make_tuple(-sqrt(3.0)*2*WHITE_NOISE_SDEV,0.0,1.0/6));
    sigmaPoints2.push_back(make_tuple(0.0,sqrt(3.0)*2*WHITE_NOISE_SDEV_Angle,1.0/6));
    sigmaPoints2.push_back(make_tuple(0.0,-sqrt(3.0)*2*WHITE_NOISE_SDEV_Angle,1.0/6));
}

DTMC::~DTMC()
{
    delete[] uStates;
}

/**
 * Get the set of states associated with the DTMC.
 * @return the set of states associated with the DTMC
 */
State_UnCtrl *DTMC::states()
{
    return uStates;
}

/*white noise along r and perpendicular to r*/
map<State_UnCtrl*,double> DTMC::getTransitionStatesAndProbs(State_UnCtrl* uState)
{
    map<State_UnCtrl*, double> TransitionStatesMapProbs;

    vector< pair<State_UnCtrl*,double> > nextStateAndProbabilities;

    for(auto sigmaPoint : sigmaPoints1)
    {
        double ra_r=get<0>(sigmaPoint);
        double ra_pr=get<1>(sigmaPoint);
        double sigmaP=get<2>(sigmaPoint);

        double r=uState->getR();
        double rv=uState->getRv();
        double theta=uState->getTheta();

        Double2D vel (rv*cos(theta*M_PI/180.0), rv*sin(theta*M_PI/180.0));
        Double2D velP (max(-UPPER_RV, min(UPPER_RV, vel.x()+ra_r)), max(-UPPER_RV, min(UPPER_RV, vel.y()+ra_pr)));
        if(velP.length()>UPPER_RV)
        {
            velP=velP.resize(UPPER_RV);
        }
        double rvP=velP.length();

        Double2D pos (r,0);
        Double2D posP (pos.x()+0.5*(vel.x()+velP.x()), pos.y()+0.5*(vel.y()+velP.y()));
        if(posP.length()>UPPER_R)
        {
            posP=posP.resize(UPPER_R);
        }
        double rP=posP.length();

        double alpha=velP.angle()-posP.angle();
        if(alpha> M_PI)
        {
            alpha= -2*M_PI +alpha;
        }
        if(alpha<-M_PI)
        {
            alpha=2*M_PI+alpha;
        }
        double thetaP = alpha*180.0/M_PI;


        int rIdxL = (int)floor(rP/rRes);
        int rvIdxL = (int)floor(rvP/rvRes);
        int thetaIdxL = (int)floor(thetaP/thetaRes);
        for(int i=0;i<=1;i++)
        {
            int rIdx = (i==0? rIdxL : rIdxL+1);
            int rIdxP= rIdx< 0? 0: (rIdx>nr? nr : rIdx);
            for(int j=0;j<=1;j++)
            {
                int rvIdx = (j==0? rvIdxL : rvIdxL+1);
                int rvIdxP= rvIdx<0? 0: (rvIdx>nrv? nrv : rvIdx);
                for(int k=0;k<=1;k++)
                {
                    int thetaIdx = (k==0? thetaIdxL : thetaIdxL+1);
                    int thetaIdxP= thetaIdx<-ntheta? -ntheta: (thetaIdx>ntheta? ntheta : thetaIdx);

                    State_UnCtrl* nextState= (uStates+State_UnCtrl::calOrder(rIdxP, rvIdxP, thetaIdxP));
                    double probability= sigmaP*(1-std::abs(rIdx-rP/rRes))*(1-std::abs(rvIdx-rvP/rvRes))*(1-std::abs(thetaIdx-thetaP/thetaRes));
                    nextStateAndProbabilities.push_back(make_pair(nextState,probability) );
                }
            }
        }

    }

    for(auto nextStateMapProb :nextStateAndProbabilities)
    {
        State_UnCtrl* nextState=nextStateMapProb.first;
        auto iPairFound = TransitionStatesMapProbs.find(nextState);
        if(iPairFound != TransitionStatesMapProbs.end())
        {
            TransitionStatesMapProbs[nextState] += nextStateMapProb.second;
        }
        else
        {
            TransitionStatesMapProbs.insert(make_pair(nextState, nextStateMapProb.second));
        }

    }

    return TransitionStatesMapProbs;
}

}

