#include "state_ctrl.h"
#include "mdp.h"

namespace acasx {

State_Ctrl::State_Ctrl(int hIdx, int oVyIdx, int iVyIdx, int raIdx)
{
    h=MDP::hRes*hIdx;
    oVy=MDP::oVRes*oVyIdx;
    iVy=MDP::iVRes*iVyIdx;
    ra=raIdx;

    int a= hIdx +MDP::nh;
    int b= oVyIdx +MDP::noVy;
    int c= iVyIdx +MDP::niVy;

    order=a*(2*MDP::noVy+1)*(2*MDP::niVy+1)*(MDP::nra)
            + b*(2*MDP::niVy+1)*(MDP::nra)
            + c*(MDP::nra)
            + ra;
}

State_Ctrl::State_Ctrl()
{
}

int State_Ctrl::calOrder(int hIdx, int oVyIdx, int iVyIdx, int raIdx)
{
    int a= hIdx +MDP::nh;
    int b= oVyIdx +MDP::noVy;
    int c= iVyIdx +MDP::niVy;

    int order=a*(2*MDP::noVy+1)*(2*MDP::niVy+1)*(MDP::nra)
            + b*(2*MDP::niVy+1)*(MDP::nra)
            + c*(MDP::nra)
            + raIdx;
    return order;
}

bool State_Ctrl::operator<(const State_Ctrl& other)
{
    return order<other.getOrder();
}

bool State_Ctrl::operator==(const State_Ctrl& other)
{
    bool result = (h==other.getH())&&(oVy==other.getoVy())&&(iVy==other.getiVy())&&(ra=other.getRa());
    return result;
}

double State_Ctrl::getH() const
{
        return h;
}


double State_Ctrl::getoVy() const
{
        return oVy;
}

double State_Ctrl::getiVy() const
{
        return iVy;
}

int State_Ctrl::getRa() const
{
        return ra;
}

int State_Ctrl::getOrder() const
{
        return order;
}


}
