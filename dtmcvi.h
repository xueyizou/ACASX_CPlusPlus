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

#ifndef DTMCVI_H
#define DTMCVI_H

//Define a Value Iteration (VI) class for solving the DTMC

#include "pch.h"

#include "state_unctrl.h"
#include "dtmc.h"

namespace acasx {

class DTMCVI
{
public:
    DTMCVI(DTMC* dtmc);
    ~DTMCVI();
    void storeValues();// store the values in a lookup table

private:
    DTMC* dtmc;
    State_UnCtrl* uStates;
    double** U;

};

}
#endif // DTMCVI_H
