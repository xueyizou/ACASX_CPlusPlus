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


#ifndef MDP_H
#define MDP_H

// define a Markov Decision Process (MDP), MDP models the dynamics of controlled states

#include "pch.h"

#include "state_ctrl.h"
#include "utils.h"

namespace acasx {

class MDP {
public:
    MDP();
    ~MDP();
    int getCStatesNum(); // get the number of controlled states
    State_Ctrl* getStates();// get the pointer to the controlled states
    vector<int> actions(int cStateOrder); // get the actions available from a controlled state
    map<int, double> getTransitionStatesAndProbs(int cStateOrder, int actionCode);//describe the transitions, return a map of next states to probabilities from a given state.
    double reward(int cStateOrder, int actionCode);// return the reward from a state by doing an action

private:    
    int numCStates;
    State_Ctrl* cStates;
};


inline int MDP::getCStatesNum()
{
    return numCStates;
}

inline State_Ctrl* MDP::getStates(){
    return cStates;
}

} /* namespace acasx */

#endif // MDP_H
