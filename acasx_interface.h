#ifndef ACASX_INTERFACE_H
#define ACASX_INTERFACE_H

#include <vector>
#include "double3d.h"

using namespace acasx;

extern void initialize();

extern int ACASX_MultiThreats(Double3D& ownshipLoc, Double3D& ownshipVel, vector<Double3D>& intrudersLocs, vector<Double3D>& intrudersVels, int lastRA);
extern int ACASX_SingleThreat(Double3D& ownshipLoc, Double3D& ownshipVel, Double3D& intruderLoc, Double3D& intruderVel, int lastRA);

extern double getActionV(int actionCode);
extern double getActionA(int actionCode);

#endif // ACASX_INTERFACE_H
