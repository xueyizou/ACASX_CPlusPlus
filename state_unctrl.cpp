#include "state_unctrl.h"

namespace acasx {

State_UnCtrl::State_UnCtrl(int rIdx, int rvIdx, int thetaIdx)
{
    r= DTMC_R_RES*rIdx;
    rv= DTMC_RV_RES*rvIdx;
    theta = DTMC_THETA_RES*thetaIdx;

    order=rIdx*(DTMC_RV_NUM+1)*(2*DTMC_THETA_NUM+1)
            + rvIdx*(2*DTMC_THETA_NUM+1)
            + (thetaIdx+DTMC_THETA_NUM);

}

State_UnCtrl::State_UnCtrl()
{
}

double State_UnCtrl::getR() const {
        return r;
    }

double State_UnCtrl::getRv() const {
    return rv;
}

double State_UnCtrl::getTheta() const {
    return theta;
}

int State_UnCtrl::getOrder() const {
    return order;
}


int State_UnCtrl::calOrder(int rIdx, int rvIdx, int thetaIdx)
{
    return State_UnCtrl_CalOrder(rIdx, rvIdx, thetaIdx);
}

}
