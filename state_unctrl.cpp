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
//uncontrolled state

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
