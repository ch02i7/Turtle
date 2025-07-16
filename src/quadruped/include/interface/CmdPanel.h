
#ifndef CMDPANEL_H
#define CMDPANEL_H

#include "message/PS_joystick.h"
#include "common/enumClass.h"
#include <pthread.h>

struct UserValue{
    float lx;
    float ly;
    float rx;
    float ry;
    float L2;
    UserValue(){
        setZero();
    }
    void setZero(){
        lx = 0;
        ly = 0;
        rx = 0;
        ry = 0;
        L2 = 0;
    }
};

class CmdPanel{
public:
    CmdPanel(){}
    virtual ~CmdPanel(){}
    UserCommand getUserCmd(){return userCmd;}
    UserValue getUserValue(){return userValue;}
    void setPassive(){userCmd = UserCommand::L2_B;}
    void setZero(){userValue.setZero();}

protected:
    UserCommand userCmd;
    UserValue userValue;
};

#endif  // CMDPANEL_H