#ifndef DTMC_HPP
#define DTMC_HPP
#include <tuple>
#include <vector>
#include <map>

using namespace std;
namespace acasx{

class State_UnCtrl;

class DTMC
{
public:
    DTMC();
    ~DTMC();
    State_UnCtrl* states();
    map<State_UnCtrl*,double > getTransitionStatesAndProbs(State_UnCtrl* ustate);

    static const double UPPER_R;
    static const double UPPER_RV;
    static const double UPPER_THETA;
    static const int nr;
    static const int nrv;
    static const int ntheta;

    static const double rRes;
    static const double rvRes;
    static const double thetaRes;

    static const double WHITE_NOISE_SDEV;
    static const double WHITE_NOISE_SDEV_Angle;

    static const int numUStates;

private:
    State_UnCtrl* uStates;

    vector<tuple<double, double, double>> sigmaPoints1;
    vector<tuple<double, double, double>> sigmaPoints2;
};

}


#endif // DTMC_HPP
