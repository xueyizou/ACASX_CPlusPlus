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

#ifndef DTMC_H
#define DTMC_H


// define a Discrete-Time Markov Chain (DTMC), DTMC models the dynamics of uncontrolled states

#include "pch.h"

#include "state_unctrl.h"
#include "double2d.h"


namespace acasx{

class DTMC
{
public:
    DTMC();
    ~DTMC();
    int getUStatesNum();//get the number of uncontrolled states
    State_UnCtrl* getStates();//get the pointer to the array of uncontrolled states

   /* get the transition (the next state and the probability) from a state.
    *input: the "from" state order
    *output: a map mapping orders of next state to the corresponding probabilities*/
    map<int, double> getTransitionStatesAndProbs(int uStateOrder);

private:
    int numUStates;
    State_UnCtrl* uStates;

};

inline int DTMC::getUStatesNum()
{
    return numUStates;
}


/**
 * Get the set of states associated with the DTMC.
 * @return the set of states associated with the DTMC
 */
inline State_UnCtrl* DTMC::getStates()
{
    return uStates;
}

}



#endif // DTMC_H
