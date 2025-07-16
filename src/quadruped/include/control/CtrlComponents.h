#ifndef CTRLCOMPONENTS_H
#define CTRLCOMPONENTS_H

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "interface/IOInterface.h"
#include "interface/CmdPanel.h"
#include "common/unitreeRobot.h"
#include "Gait/WaveGenerator.h"
#include "control/Estimator.h"
#include "control/BalanceCtrl.h"
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"  

struct CtrlComponents{
public:
    CtrlComponents()
    {
        lowCmd = new LowlevelCmd();  
        lowState = new LowlevelState();
        lowCmd_xm = new LowlevelCmd_XM();  
        lowState_xm = new LowlevelState_XM();
        contact = new VecInt4;
        phase = new Vec4;
        *contact = VecInt4(0, 0, 0, 0);
        *phase = Vec4(0.5, 0.5, 0.5, 0.5);
    }
    ~CtrlComponents(){
        delete lowCmd;
        delete lowState;
        delete lowCmd_xm;
        delete lowState_xm;        
        delete ioInter;
        delete robotModel;
        delete legModel;
        delete waveGen;
        delete estimator;
        delete balCtrl;
    }
    LowlevelCmd *lowCmd; 
    LowlevelState *lowState;  
    LowlevelCmd_XM *lowCmd_xm; 
    LowlevelState_XM *lowState_xm;  
    IOInterface *ioInter;
    QuadrupedRobot *robotModel;
    QuadrupedLeg *legModel;     // 四足腿运动学模型
    WaveGenerator *waveGen;
    Estimator *estimator;
    BalanceCtrl *balCtrl;

    VecInt4 *contact;
    Vec4 *phase;

    double dt;
    bool *running;
    CtrlPlatform ctrlPlatform;



    void sendRecv(){
        ioInter->sendRecv_xm(lowCmd_xm, lowState_xm,1);
    }

    void sendRecv_set(){
        ioInter->sendRecv_xm(lowCmd_xm, lowState_xm,2);
    }

    void runWaveGen(){
        waveGen->calcContactPhase(*phase, *contact, _waveStatus);
    }

    void setAllStance(){
        _waveStatus = WaveStatus::STANCE_ALL;
    }

    void setAllSwing(){
        _waveStatus = WaveStatus::SWING_ALL;
    }

    void setStartWave(){
        _waveStatus = WaveStatus::WAVE_ALL;
    }

    void geneObj(){
        estimator = new Estimator(robotModel, lowState, contact, phase, dt);
        balCtrl = new BalanceCtrl(robotModel);

    }


private:
    WaveStatus _waveStatus = WaveStatus::SWING_ALL;
};

#endif  // CTRLCOMPONENTS_H