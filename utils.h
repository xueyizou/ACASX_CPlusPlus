#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <limits>
#include "constants.h"

using namespace std;
namespace acasx {

extern int State_UnCtrl_CalOrder(int rIdx, int rvIdx, int thetaIdx);
extern int State_Ctrl_CalOrder(int hIdx, int oVyIdx, int iVyIdx, int raIdx);

extern double getActionV(int actionCode);
extern double getActionA(int actionCode);

}


#endif // UTILS_H

