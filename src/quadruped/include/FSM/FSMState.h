/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef FSMSTATE_H
#define FSMSTATE_H

#include <string>
#include <iostream>
#include <unistd.h>
#include "control/CtrlComponents.h"
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "common/enumClass.h"
#include "common/mathTools.h"
#include "common/mathTypes.h"
#include "common/timeMarker.h"
#include "interface/CmdPanel.h"

class FSMState{
public:
    FSMState(CtrlComponents *ctrlComp, FSMStateName stateName, std::string stateNameString);

    virtual void enter() = 0;
    virtual void run() = 0;
    virtual void exit() = 0;
    virtual FSMStateName checkChange() {return FSMStateName::INVALID;}

    FSMStateName _stateName;
    std::string _stateNameString;
protected:
    CtrlComponents *_ctrlComp;
    FSMStateName _nextStateName;

    LowlevelCmd *_lowCmd;
    LowlevelCmd_XM *_lowCmd_XM;
    LowlevelState *_lowState;
    LowlevelState_XM *_lowState_XM;
    UserValue _userValue;
    int countt;
};

#endif  // FSMSTATE_H