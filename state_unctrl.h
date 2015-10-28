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

#ifndef STATE_UNCTRL_H
#define STATE_UNCTRL_H

//uncontrolled state

#include "pch.h"

#include "utils.h"

namespace acasx {

class State_UnCtrl
{
public:
    State_UnCtrl(int rIdx, int rvIdx, int thetaIdx);
    State_UnCtrl();
    double getR() const;
    double getRv() const;
    double getTheta() const;
    int getOrder() const;

    static int calOrder(int rIdx, int rvIdx, int thetaIdx);

private:
    double r;
    double rv;
    double theta;
    int order;

};

}


#endif // STATE_UNCTRL_H
