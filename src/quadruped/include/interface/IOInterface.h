
#ifndef IOINTERFACE_H
#define IOINTERFACE_H

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "interface/CmdPanel.h"
#include <string>

class IOInterface{
public:
IOInterface(){}
~IOInterface(){delete cmdPanel;}
virtual void sendRecv(const LowlevelCmd *cmd, LowlevelState *state, int id) = 0;

virtual void sendRecv_xm(const LowlevelCmd_XM *cmd, LowlevelState_XM *state, int id) = 0;
void zeroCmdPanel(){cmdPanel->setZero();}
void setPassive(){cmdPanel->setPassive();}

protected:
CmdPanel *cmdPanel;
};

#endif  //IOINTERFACE_H