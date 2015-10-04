#ifndef MDPVI_HPP
#define MDPVI_HPP
#include <vector>

using namespace std;
namespace acasx {

class MDP;
class State_Ctrl;

class MDPVI
{
public:
    MDPVI(MDP* mdp);
    ~MDPVI();
    double getQValue(int k,State_Ctrl* state, int action);
    void storeQValues();

    static const int T;

private:
    MDP* mdp;
    State_Ctrl* states;
    double** U;//U[T+1] is for the K-step expected cost when k>T (i.e. k>K), denoted JkBar

};


}

#endif // MDPVI_HPP
