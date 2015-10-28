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

// controlled state

#include "state_ctrl.h"

namespace acasx {

State_Ctrl::State_Ctrl(int hIdx, int oVyIdx, int iVyIdx, int raIdx)
{
    h=MDP_H_RES*hIdx;
    oVy=MDP_OV_RES*oVyIdx;
    iVy=MDP_IV_RES*iVyIdx;
    ra=raIdx;

    int a= hIdx +MDP_H_NUM;
    int b= oVyIdx +MDP_OVY_NUM;
    int c= iVyIdx +MDP_IVY_NUM;

    order=a*(2*MDP_OVY_NUM+1)*(2*MDP_IVY_NUM+1)*(MDP_RA_NUM)
            + b*(2*MDP_IVY_NUM+1)*(MDP_RA_NUM)
            + c*(MDP_RA_NUM)
            + ra;
}

State_Ctrl::State_Ctrl()
{
}

int State_Ctrl::calOrder(int hIdx, int oVyIdx, int iVyIdx, int raIdx)
{
    return State_Ctrl_CalOrder(hIdx, oVyIdx, iVyIdx,  raIdx);
}

bool State_Ctrl::operator<(const State_Ctrl& other)
{
    return order<other.getOrder();
}

bool State_Ctrl::operator==(const State_Ctrl& other)
{
    bool result = (h==other.getH())&&(oVy==other.getoVy())&&(iVy==other.getiVy())&&(ra=other.getRa());
    return result;
}

double State_Ctrl::getH() const
{
        return h;
}


double State_Ctrl::getoVy() const
{
        return oVy;
}

double State_Ctrl::getiVy() const
{
        return iVy;
}

int State_Ctrl::getRa() const
{
        return ra;
}

int State_Ctrl::getOrder() const
{
        return order;
}


}
