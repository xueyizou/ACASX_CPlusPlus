#include "dtmc.h"

using namespace std;

namespace acasx {

vector< tuple<double, double, double> > sigmaPoints1;
vector< tuple<double, double, double> > sigmaPoints2;

DTMC::DTMC()
{
    numUStates= (DTMC_R_NUM+1)*(DTMC_RV_NUM+1)*(2*DTMC_THETA_NUM+1);
    uStates = new State_UnCtrl[numUStates];
    if(!uStates)
    {
        cout<<uStates<<endl;
        cerr<<"failure in dynamic memory allocation "<<endl;
        return;
    }

    for(int rIdx=0; rIdx<=DTMC_R_NUM; ++rIdx)//
    {
        for(int rvIdx=0; rvIdx<=DTMC_RV_NUM; ++rvIdx)//
        {
            for(int thetaIdx=-DTMC_THETA_NUM; thetaIdx<=DTMC_THETA_NUM; ++thetaIdx)
            {
                State_UnCtrl* ptr = new State_UnCtrl(rIdx, rvIdx, thetaIdx);
                uStates[State_UnCtrl::calOrder(rIdx, rvIdx, thetaIdx)]=*ptr;
                delete ptr;
            }
        }
    }

    sigmaPoints1.push_back(make_tuple(0.0,0.0,1.0/3));
    sigmaPoints1.push_back(make_tuple(sqrt(3.0)*2*DTMC_WHITE_NOISE_SDEV,0.0,1.0/6));
    sigmaPoints1.push_back(make_tuple(-sqrt(3.0)*2*DTMC_WHITE_NOISE_SDEV,0.0,1.0/6));
    sigmaPoints1.push_back(make_tuple(0.0,sqrt(3.0)*2*DTMC_WHITE_NOISE_SDEV,1.0/6));
    sigmaPoints1.push_back(make_tuple(0.0,-sqrt(3.0)*2*DTMC_WHITE_NOISE_SDEV,1.0/6));

    sigmaPoints2.push_back(make_tuple(0.0,0.0,1.0/3));
    sigmaPoints2.push_back(make_tuple(sqrt(3.0)*2*DTMC_WHITE_NOISE_SDEV,0.0,1.0/6));
    sigmaPoints2.push_back(make_tuple(-sqrt(3.0)*2*DTMC_WHITE_NOISE_SDEV,0.0,1.0/6));
    sigmaPoints2.push_back(make_tuple(0.0,sqrt(3.0)*2*DTMC_WHITE_NOISE_SDEV_ANGLE,1.0/6));
    sigmaPoints2.push_back(make_tuple(0.0,-sqrt(3.0)*2*DTMC_WHITE_NOISE_SDEV_ANGLE,1.0/6));
}

DTMC::~DTMC()
{
    delete[] uStates;
}



/*white noise along r and perpendicular to r*/
map<int,double> DTMC::getTransitionStatesAndProbs(int uStateOrder)
{
    State_UnCtrl* uStatePtr = uStates+uStateOrder;

    map<int, double> TransitionStatesMapProbs;

    vector< pair<int,double> > nextStateAndProbabilities;

    for(auto sigmaPoint : sigmaPoints1)
    {
        double ra_r=get<0>(sigmaPoint);
        double ra_pr=get<1>(sigmaPoint);
        double sigmaP=get<2>(sigmaPoint);

        double r=uStatePtr->getR();
        double rv=uStatePtr->getRv();
        double theta=uStatePtr->getTheta();

        Double2D vel (rv*cos(theta*M_PI/180.0), rv*sin(theta*M_PI/180.0));
        Double2D velP (max(-DTMC_UPPER_RV, min(DTMC_UPPER_RV, vel.x()+ra_r)), max(-DTMC_UPPER_RV, min(DTMC_UPPER_RV, vel.y()+ra_pr)));
        if(velP.length()>DTMC_UPPER_RV)
        {
            velP=velP.resize(DTMC_UPPER_RV);
        }
        double rvP=velP.length();

        Double2D pos (r,0);
        Double2D posP (pos.x()+0.5*(vel.x()+velP.x()), pos.y()+0.5*(vel.y()+velP.y()));
        if(posP.length()>DTMC_UPPER_R)
        {
            posP=posP.resize(DTMC_UPPER_R);
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


        int rIdxL = (int)floor(rP/DTMC_R_RES);
        int rvIdxL = (int)floor(rvP/DTMC_RV_RES);
        int thetaIdxL = (int)floor(thetaP/DTMC_THETA_RES);
        for(int i=0;i<=1;i++)
        {
            int rIdx = (i==0? rIdxL : rIdxL+1);
            int rIdxP= rIdx< 0? 0: (rIdx>DTMC_R_NUM? DTMC_R_NUM : rIdx);
            for(int j=0;j<=1;j++)
            {
                int rvIdx = (j==0? rvIdxL : rvIdxL+1);
                int rvIdxP= rvIdx<0? 0: (rvIdx>DTMC_RV_NUM? DTMC_RV_NUM : rvIdx);
                for(int k=0;k<=1;k++)
                {
                    int thetaIdx = (k==0? thetaIdxL : thetaIdxL+1);
                    int thetaIdxP= thetaIdx<-DTMC_THETA_NUM? -DTMC_THETA_NUM: (thetaIdx>DTMC_THETA_NUM? DTMC_THETA_NUM : thetaIdx);

                    int nextStateOrder= State_UnCtrl::calOrder(rIdxP, rvIdxP, thetaIdxP);
                    double probability= sigmaP*(1-fabs(rIdx-rP/DTMC_R_RES))*(1-fabs(rvIdx-rvP/DTMC_RV_RES))*(1-std::fabs(thetaIdx-thetaP/DTMC_THETA_RES));
                    nextStateAndProbabilities.push_back(make_pair(nextStateOrder,probability) );
                }
            }
        }

    }

    for(auto nextStateMapProb :nextStateAndProbabilities)
    {
        int nextStateOrder=nextStateMapProb.first;
        auto iPairFound = TransitionStatesMapProbs.find(nextStateOrder);
        if(iPairFound != TransitionStatesMapProbs.end())
        {
            TransitionStatesMapProbs[nextStateOrder] += nextStateMapProb.second;
        }
        else
        {
            TransitionStatesMapProbs.insert(nextStateMapProb);
        }

    }

    return TransitionStatesMapProbs;
}

}

