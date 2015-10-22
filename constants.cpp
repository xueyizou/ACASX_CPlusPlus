#include "constants.h"

namespace acasx{

const int TIME_HORIZON=20;

const double DTMC_UPPER_R = 12200;
const double DTMC_UPPER_RV = 610;
const double DTMC_UPPER_THETA=180.0;

const int DTMC_R_NUM = 61;
const int DTMC_RV_NUM= 61;
const int DTMC_THETA_NUM= 36;

const double DTMC_R_RES = DTMC_UPPER_R/DTMC_R_NUM;
const double DTMC_RV_RES = DTMC_UPPER_RV/DTMC_RV_NUM;
const double DTMC_THETA_RES = DTMC_UPPER_THETA/DTMC_THETA_NUM;

const double DTMC_WHITE_NOISE_SDEV=3.0;
const double DTMC_WHITE_NOISE_SDEV_ANGLE=2.0;


const double DTMCVI_COLLISION_R=500;


const double MDP_UPPER_H=600.0;
const double MDP_UPPER_VY=70.0;

const int MDP_H_NUM = 10;//10
const int MDP_OVY_NUM=7;//7
const int MDP_IVY_NUM= 7;//7
const int MDP_RA_NUM=7;

const double MDP_H_RES = MDP_UPPER_H/MDP_H_NUM;
const double MDP_OV_RES = MDP_UPPER_VY/MDP_OVY_NUM;
const double MDP_IV_RES = MDP_UPPER_VY/MDP_IVY_NUM;

const double MDP_WHITE_NOISE_SDEV=3.0;

}
