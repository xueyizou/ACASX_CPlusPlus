#ifndef MDPVI_H
#define MDPVI_H

#include "pch.h"

#include "mdp.h"
#include "state_ctrl.h"

namespace acasx {

class MDPVI
{
public:
    MDPVI(MDP* mdp);
    ~MDPVI();
    double getQValue(int k, int stateOrder, int action);
    void storeQValues();

private:
    MDP* mdp;
    State_Ctrl* cStates;
    double** U;//U[T+1] is for the K-step expected cost when k>T (i.e. k>K), denoted JkBar

};


}

#endif // MDPVI_H
