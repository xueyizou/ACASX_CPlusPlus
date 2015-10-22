#ifndef DTMC_H
#define DTMC_H

#include "pch.h"

#include "state_unctrl.h"
#include "double2d.h"


namespace acasx{

class DTMC
{
public:
    DTMC();
    ~DTMC();
    int getUStatesNum();
    State_UnCtrl* getStates();
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
