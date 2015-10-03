#ifndef MDP_H
#define MDP_H


#include <vector>
#include <iostream>
#include <map>
#include <tuple>

using namespace std;

namespace acasx {

class State_Ctrl;

//template <typename T>
//struct PredComparingPoint
//{
//    bool operator()(const T ptr1, const T ptr2)
//    {
//        return ptr1->getOrder() < ptr2->getOrder();
//    }
//};

class MDP {
public:
    MDP();
    virtual ~MDP();
    State_Ctrl* states(void);
    vector<int> actions(State_Ctrl* cstate);
    map<State_Ctrl*,double > getTransitionStatesAndProbs(State_Ctrl* cstate, int actionCode);
    double reward(State_Ctrl* cstate,int actionCode);


    static const double UPPER_H;
    static const double UPPER_VY;

    static const int nh;//10
    static const int noVy;//7
    static const int niVy;//7
    static const int nra;

    static const double hRes;
    static const double oVRes;
    static const double iVRes;

    static const double WHITE_NOISE_SDEV;
    static const int numCStates;

private:    
    State_Ctrl* cStates;
    vector<tuple<double, double, double>> sigmaPointsA;
    vector<tuple<double, double, double>> sigmaPointsB;

};

} /* namespace acasx */

#endif // MDP_H
