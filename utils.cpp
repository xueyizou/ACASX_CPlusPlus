#include "utils.h"

namespace acasx {

int State_UnCtrl_CalOrder(int rIdx, int rvIdx, int thetaIdx)
{
    int order=rIdx*(DTMC_RV_NUM+1)*(2*DTMC_THETA_NUM+1)
            + rvIdx*(2*DTMC_THETA_NUM+1)
            + (thetaIdx+DTMC_THETA_NUM);

    return order;
}

int State_Ctrl_CalOrder(int hIdx, int oVyIdx, int iVyIdx, int raIdx)
{
    int a= hIdx +MDP_H_NUM;
    int b= oVyIdx +MDP_OVY_NUM;
    int c= iVyIdx +MDP_IVY_NUM;

    int order=a*(2*MDP_OVY_NUM+1)*(2*MDP_IVY_NUM+1)*(MDP_RA_NUM)
            + b*(2*MDP_IVY_NUM+1)*(MDP_RA_NUM)
            + c*(MDP_RA_NUM)
            + raIdx;
    return order;
}

double getActionV(int actionCode)
{
    if(actionCode==-1 )//"Loop"
    {
        cerr<<"no V in Loop action!"<<endl;
        return numeric_limits<double>::quiet_NaN();
    }
    else if(actionCode==0)//"COC"
    {
        return numeric_limits<double>::quiet_NaN();
    }
    else if(actionCode==1 || actionCode==3) //"CL25" "SCL25"
    {
        return 25;
    }
    else if(actionCode==2 || actionCode==4)//"DES25" "SDES25"
    {
        return -25;
    }
    else if(actionCode==5)//"SCL42"
    {
        return 42;
    }
    else if(actionCode==6)//"SDES42"
    {
        return -42;
    }
    else
    {
        cerr<<"error happens in ACASXUtils.getActionV(int actionCode):an unknown aciton."<<endl;
        return numeric_limits<double>::quiet_NaN();
    }
}


double getActionA(int actionCode)
{
    if(actionCode==-1)//"Loop"
    {
        cerr<<"no A in Loop action!"<<endl;
        return numeric_limits<double>::quiet_NaN();
    }
    else if(actionCode==0)//"COC"
    {
        return 0.0;
    }
    else if(actionCode==1 )//"CL25"
    {
        return 8;
    }
    else if(actionCode==2 )//"DES25"
    {
        return -8;
    }
    else if(actionCode==3 || actionCode==5)//"SCL42" "SCL25"
    {
        return 10.7;
    }
    else if(actionCode==4|| actionCode==6)//"SDES42" "SDES25"
    {
        return -10.7;
    }
    else
    {
        cerr<<"error happens in ACASXUtils.getActionA(int actionCode):an unknown aciton."<<endl;
        return numeric_limits<double>::quiet_NaN();
    }
}

}
