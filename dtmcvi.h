#ifndef DTMCVI_H
#define DTMCVI_H

#include "pch.h"

#include "state_unctrl.h"
#include "dtmc.h"

namespace acasx {

class DTMCVI
{
public:
    DTMCVI(DTMC* dtmc);
    ~DTMCVI();
    void storeValues();

private:
    DTMC* dtmc;
    State_UnCtrl* uStates;
    double** U;

};

}
#endif // DTMCVI_H
