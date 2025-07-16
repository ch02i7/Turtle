
#include "FSM/FSMState.h"

FSMState::FSMState(CtrlComponents *ctrlComp, FSMStateName stateName, std::string stateNameString)
            :_ctrlComp(ctrlComp), _stateName(stateName), _stateNameString(stateNameString){
    _lowCmd = _ctrlComp->lowCmd;
    _lowState = _ctrlComp->lowState;
    _lowCmd_XM = _ctrlComp->lowCmd_xm;
    _lowState_XM = _ctrlComp->lowState_xm;
}