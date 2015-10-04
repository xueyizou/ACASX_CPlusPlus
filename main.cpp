#include "mdp.h"
#include "mdpvi.hpp"
#include <iostream>
#include <chrono>
#include <ctime>
#include "dtmc.hpp"
#include "dtmcvi.hpp"


using namespace acasx;
using namespace std;

int main(int argc, char *argv[])
{
    cout<<"hi!Process starts..."<<endl;

    std::chrono::time_point<std::chrono::system_clock> time0, time1,time2,time3, time4, time5, time6, time7;

    time0 = std::chrono::system_clock::now();
    MDP* mdp = new MDP();
    time1 = std::chrono::system_clock::now();
    cout<<"MDP building Done! The running time is "<< (static_cast<std::chrono::duration<double>>(time1-time0)).count() <<" senconds."<<endl;
    MDPVI* mdpVI = new MDPVI(mdp);
    time2 =std::chrono::system_clock::now();
    cout<<"MDP Value Iteration Done! The running time is "<< (static_cast<std::chrono::duration<double>>(time2-time1)).count() <<" senconds."<<endl;
    mdpVI->storeQValues();
    time3 = std::chrono::system_clock::now();
    cout<<"MDP QValue Store Done! The running time is "<< (static_cast<std::chrono::duration<double>>(time3-time2)).count() <<" senconds."<<endl;
    delete mdp;
    delete mdpVI;

    time4 = std::chrono::system_clock::now();
    DTMC* dtmc = new DTMC();
    time5 = std::chrono::system_clock::now();
    cout<<"DTMC building Done! The running time is "<< (static_cast<std::chrono::duration<double>>(time5-time4)).count() <<" senconds."<<endl;
    DTMCVI* dtmcVI = new DTMCVI(dtmc);
    time6 = std::chrono::system_clock::now();
    cout<<"DTMC Value Iteration Done! The running time is "<< (static_cast<std::chrono::duration<double>>(time6-time5)).count() <<" senconds."<<endl;
    dtmcVI->storeValues();
    time7 = std::chrono::system_clock::now();
    cout<<"DTMC values Storage Done! The running time is "<< (static_cast<std::chrono::duration<double>>(time7-time6)).count()<<" senconds."<<endl;
    delete dtmc;
    delete dtmcVI;

    return 0;
}

