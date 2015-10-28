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

