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

#ifndef STATE_CTRL_H
#define STATE_CTRL_H

// controlled state

#include "pch.h"

#include "utils.h"

namespace acasx {

class State_Ctrl {
public:
    State_Ctrl(int hIdx, int oVyIdx, int iVyIdx, int raIdx);
    State_Ctrl();
    double getH() const;
    double getoVy() const;
    double getiVy() const;
    int getRa() const;
    int getOrder() const;
    bool operator<(const State_Ctrl& other);
    bool operator==(const State_Ctrl& other);

    static int calOrder(int hIdx, int oVyIdx, int iVyIdx, int raIdx);

private:
    double h;
    double oVy;
    double iVy;
    int ra;
    int order;
};

} /* namespace acasx */

#endif // STATE_CTRL_H
