#ifndef DTMCVI_HPP
#define DTMCVI_HPP

#include <string>
using namespace std;

namespace acasx {
class DTMC;
class State_UnCtrl;

class DTMCVI
{
public:
    DTMCVI(DTMC* dtmc);
    ~DTMCVI();
    void storeValues();

    static const int T;
    static  const double COLLISION_R;

private:
    DTMC* dtmc;
    State_UnCtrl* uStates;
    double** U;

};

}
#endif // DTMCVI_HPP
