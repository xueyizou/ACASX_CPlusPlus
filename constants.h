#ifndef CONSTANTS_H
#define CONSTANTS_H

namespace acasx{

extern const int TIME_HORIZON;

extern const double DTMC_UPPER_R;
extern const double DTMC_UPPER_RV;
extern const double DTMC_UPPER_THETA;

extern const int DTMC_R_NUM;
extern const int DTMC_RV_NUM;
extern const int DTMC_THETA_NUM;

extern const double DTMC_R_RES;
extern const double DTMC_RV_RES;
extern const double DTMC_THETA_RES;

extern const double DTMC_WHITE_NOISE_SDEV;
extern const double DTMC_WHITE_NOISE_SDEV_ANGLE;


extern const double DTMCVI_COLLISION_R;


extern const double MDP_UPPER_H;
extern const double MDP_UPPER_VY;

extern const int MDP_H_NUM;
extern const int MDP_OVY_NUM;
extern const int MDP_IVY_NUM;
extern const int MDP_RA_NUM;

extern const double MDP_H_RES;
extern const double MDP_OV_RES;
extern const double MDP_IV_RES;

extern const double MDP_WHITE_NOISE_SDEV;

}

#endif // CONSTANTS_H

