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

#ifndef ACASX_INTERFACE_H
#define ACASX_INTERFACE_H


//The interface between applications and the ACAS X collision avoidance logic.

#include <vector>
#include "double3d.h"

using namespace acasx;

//used only once to load the lookup table, which is time consuming.
extern void initialize();

//for only one threat, passing in the ownship and the intruder's positions and velocities and the recent Resolution Advice (RA) issured by the ownship
extern int ACASX_SingleThreat(Double3D& ownshipLoc, Double3D& ownshipVel, Double3D& intruderLoc, Double3D& intruderVel, int lastRA);

//for multiple threat, passing in the ownship and the intruders' positions and velocities as vectors, and the recent Resolution Advice (RA) issured by the ownship
extern int ACASX_MultiThreats(Double3D& ownshipLoc, Double3D& ownshipVel, vector<Double3D>& intrudersLocs, vector<Double3D>& intrudersVels, int lastRA);

//for getting the target velocity of a Resolution Advice (RA)
extern double getActionV(int actionCode);

//for getting the target acceleration of a Resolution Advice (RA)
extern double getActionA(int actionCode);

#endif // ACASX_INTERFACE_H
