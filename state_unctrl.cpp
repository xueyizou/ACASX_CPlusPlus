#include "state_unctrl.hpp"
#include "dtmc.hpp"

using namespace std;

namespace acasx {

State_UnCtrl::State_UnCtrl(int rIdx, int rvIdx, int thetaIdx)
{
    r= DTMC::rRes*rIdx;
    rv= DTMC::rvRes*rvIdx;
    theta = DTMC::thetaRes*thetaIdx;

    order=rIdx*(DTMC::nrv+1)*(2*DTMC::ntheta+1)
            + rvIdx*(2*DTMC::ntheta+1)
            + (thetaIdx+DTMC::ntheta);

}

State_UnCtrl::State_UnCtrl()
{
}

double State_UnCtrl::getR() const {
        return r;
    }

double State_UnCtrl::getRv() const {
    return rv;
}

double State_UnCtrl::getTheta() const {
    return theta;
}

int State_UnCtrl::getOrder() const {
    return order;
}


int State_UnCtrl::calOrder(int rIdx, int rvIdx, int thetaIdx)
{
    int order=rIdx*(DTMC::nrv+1)*(2*DTMC::ntheta+1)
            + rvIdx*(2*DTMC::ntheta+1)
            + (thetaIdx+DTMC::ntheta);

    return order;
}

}
