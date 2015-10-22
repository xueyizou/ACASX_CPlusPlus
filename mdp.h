#ifndef MDP_H
#define MDP_H

#include "pch.h"

#include "state_ctrl.h"
#include "utils.h"

namespace acasx {

class MDP {
public:
    MDP();
    ~MDP();
    int getCStatesNum();
    State_Ctrl* getStates();
    vector<int> actions(int cStateOrder);
    map<int, double> getTransitionStatesAndProbs(int cStateOrder, int actionCode);
    double reward(int cStateOrder, int actionCode);

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
