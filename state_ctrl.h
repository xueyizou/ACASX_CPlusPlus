#ifndef STATE_CTRL_H
#define STATE_CTRL_H

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
