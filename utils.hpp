#ifndef UTILS_HPP
#define UTILS_HPP

#include <limits>

using namespace std;

namespace acasx {

double getActionV(int actionCode)
{
    if(actionCode==-1 )//"Loop"
    {
        cerr<<"no V in Loop action!"<<endl;
        return nan("");
    }
    else if(actionCode==0)//"COC"
    {
        return nan("");
    }
    else if(actionCode==1 || actionCode==3) //"CL25" "SCL25"
    {
        return 25;
    }
    else if(actionCode==2 || actionCode==4)//"DES25" "SDES25"
    {
        return -25;
    }
    else if(actionCode==5)//"SCL42"
    {
        return 42;
    }
    else if(actionCode==6)//"SDES42"
    {
        return -42;
    }
    else
    {
        cerr<<"error happens in ACASXUtils.getActionV(int actionCode):an unknown aciton."<<endl;
        return nan("");
    }
}


double getActionA(int actionCode)
{
    if(actionCode==-1)//"Loop"
    {
        cerr<<"no A in Loop action!"<<endl;
        return nan("");
    }
    else if(actionCode==0)//"COC"
    {
        return 0.0;
    }
    else if(actionCode==1 )//"CL25"
    {
        return 8;
    }
    else if(actionCode==2 )//"DES25"
    {
        return -8;
    }
    else if(actionCode==3 || actionCode==5)//"SCL42" "SCL25"
    {
        return 10.7;
    }
    else if(actionCode==4|| actionCode==6)//"SDES42" "SDES25"
    {
        return -10.7;
    }
    else
    {
        cerr<<"error happens in ACASXUtils.getActionA(int actionCode):an unknown aciton."<<endl;
        return nan("");
    }
}

}


#endif // UTILS_HPP

