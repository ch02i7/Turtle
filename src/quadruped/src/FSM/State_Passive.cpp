
#include "FSM/State_Passive.h"
#include "interface/CmdPanel.h" 
#include "rclcpp/rclcpp.hpp"

State_Passive::State_Passive(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::PASSIVE, "passive"){}

void State_Passive::enter(){
    if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
        for(int i=0; i<12; i++){
            _lowCmd->motorCmd[i].mode = 10;
            _lowCmd->motorCmd[i].q = 0;
            _lowCmd->motorCmd[i].dq = 0;
            _lowCmd->motorCmd[i].Kp = 0;
            _lowCmd->motorCmd[i].Kd = 8;
            _lowCmd->motorCmd[i].tau = 0;
        }
    }
    else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
        for(int i=0; i<12; i++){
            _lowCmd->motorCmd[i].mode = 10;
            _lowCmd->motorCmd[i].q = 0;
            _lowCmd->motorCmd[i].dq = 0;
            _lowCmd->motorCmd[i].Kp = 0;
            _lowCmd->motorCmd[i].Kd = 3;
            _lowCmd->motorCmd[i].tau = 0;
        }

    }

    _ctrlComp->setAllSwing();
    _ctrlComp->sendRecv_set();
}

void State_Passive::run(){

    if(countt <100){
        _lowCmd_XM->motorCmd[1].q = (0.4*countt - 0.2*(100-countt))/100;
        _lowCmd_XM->motorCmd[2].q = -0.9;
        _lowCmd_XM->motorCmd[5].q = (-0.6*countt + 0.0*(100-countt))/100;

        _lowCmd_XM->motorCmd[8].q = (0.4*countt - 0.2*(100-countt))/100;
        _lowCmd_XM->motorCmd[9].q = 0.9;
        _lowCmd_XM->motorCmd[12].q = (-0.6*countt + 0.0*(100-countt))/100;

        countt++;

    }else if(countt>=100 && countt<200){
        _lowCmd_XM->motorCmd[0].q = (-0.4*(countt-100) + 0.4*(100-(countt-100)))/100;
        _lowCmd_XM->motorCmd[3].q = (-0.4*(countt-100) + 0.0*(100-(countt-100)))/100;
        _lowCmd_XM->motorCmd[4].q = (0.3*(countt-100) + 0.0*(100-(countt-100)))/100;

        _lowCmd_XM->motorCmd[7].q = (0.4*(countt-100) - 0.4*(100-(countt-100)))/100;
        _lowCmd_XM->motorCmd[10].q = (0.4*(countt-100) + 0.0*(100-(countt-100)))/100;
        _lowCmd_XM->motorCmd[11].q = (-0.3*(countt-100) + 0.0*(100-(countt-100)))/100;

        countt++;

    }else if(countt>=200 && countt<300){
        _lowCmd_XM->motorCmd[1].q = (-0.2*(countt-200) + 0.4*(100-(countt-200)))/100;
        _lowCmd_XM->motorCmd[5].q = (0.0*(countt-200) - 0.6*(100-(countt-200)))/100;

        _lowCmd_XM->motorCmd[8].q = (-0.2*(countt-200) + 0.4*(100-(countt-200)))/100;
        _lowCmd_XM->motorCmd[12].q = (0.0*(countt-200) - 0.6*(100-(countt-200)))/100;
 
        countt++;       

    }else if(countt>=300 && countt<400){
        _lowCmd_XM->motorCmd[0].q = (0.4*(countt-300) - 0.4*(100-(countt-300)))/100;
        _lowCmd_XM->motorCmd[3].q = (0*(countt-300) - 0.4*(100-(countt-300)))/100;
        _lowCmd_XM->motorCmd[4].q = (0*(countt-300) + 0.3*(100-(countt-300)))/100;

        _lowCmd_XM->motorCmd[7].q = (-0.4*(countt-300) + 0.4*(100-(countt-300)))/100;
        _lowCmd_XM->motorCmd[10].q = (0*(countt-300) + 0.4*(100-(countt-300)))/100;
        _lowCmd_XM->motorCmd[11].q = (0*(countt-300) - 0.3*(100-(countt-300)))/100;

        countt++;

    }else {
        countt = 0;
    }

}

void State_Passive::exit(){

}

FSMStateName State_Passive::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::PASSIVE;
    }
}