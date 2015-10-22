#ifndef STATE_UNCTRL_H
#define STATE_UNCTRL_H

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
