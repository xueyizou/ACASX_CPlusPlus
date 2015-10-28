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

#ifndef MDPVI_H
#define MDPVI_H


//Define a Value Iteration (VI) class for solving the MDP

#include "pch.h"

#include "mdp.h"
#include "state_ctrl.h"

namespace acasx {

class MDPVI
{
public:
    MDPVI(MDP* mdp);
    ~MDPVI();
    double getQValue(int k, int stateOrder, int action);//q-value: the expected accumulated reward from a state after committed to a certain action, and them acting optimally
    void storeQValues();// store the q-values in lookup tables

private:
    MDP* mdp;
    State_Ctrl* cStates;
    double** U;//U[T+1] is for the K-step expected cost when k>T (i.e. k>K), denoted JkBar

};


}

#endif // MDPVI_H
