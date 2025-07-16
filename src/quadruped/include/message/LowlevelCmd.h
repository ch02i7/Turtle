/**********************************************************************
 底层控制指令数据结构
 功能：封装发送给电机驱动器的原始控制指令
 包含：12个关节电机的目标位置、速度、力矩及控制参数
***********************************************************************/
#ifndef LOWLEVELCMD_H
#define LOWLEVELCMD_H

#include "common/mathTypes.h"
#include "common/mathTools.h"

// 单个电机控制指令（与驱动器协议对齐）
struct MotorCmd{
    unsigned int mode; // 控制模式 (0: 空闲, 10: 位置控制)
    float q;          // 目标关节位置 (rad)
    float dq;         // 目标关节速度 (rad/s)
    float tau;        // 目标关节力矩 (Nm)
    float Kp;         // 位置环比例增益
    float Kd;         // 速度环微分增益

    MotorCmd(){        // 构造函数，初始化成员变量
        mode = 0;     // 初始化为空闲模式
        q = 0;        // 零位置
        dq = 0;       // 零速度
        tau = 0;      // 零力矩
        Kp = 0;       // 零增益
        Kd = 0;
    }
};


// 整机底层控制指令（四足机器人16个关节）
struct LowlevelCmd_XM{
    MotorCmd motorCmd[16];// 四腿控制指令（每腿4关节 x 4腿）

    // 设置全部16个关节目标位置    
    void setQ(Vec16 q){
        for(int i(0); i<16; ++i){
            motorCmd[i].q = q(i);// 按索引顺序设置
        }
    }

    // 设置单腿四个关节目标位置
    void setQ(int legID, Vec4 qi){
        motorCmd[legID*4+0].q = qi(0);
        motorCmd[legID*4+1].q = qi(1);
        motorCmd[legID*4+2].q = qi(2);
        motorCmd[legID*4+3].q = qi(3);
    }

    // 设置全部16个关节目标速度       
    void setQd(Vec16 qd){
        for(int i(0); i<16; ++i){
            motorCmd[i].dq = qd(i);
        }
    }

    // 设置单腿四个关节目标速度    
    void setQd(int legID, Vec4 qdi){
        motorCmd[legID*4+0].dq = qdi(0);
        motorCmd[legID*4+1].dq = qdi(1);
        motorCmd[legID*4+2].dq = qdi(2);
        motorCmd[legID*4+3].dq = qdi(3);
    }

    // 设置关节力矩（带力矩限幅）      
    void setTau(Vec16 tau, Vec2 torqueLimit = Vec2(-50, 50)){
        for(int i(0); i<16; ++i){
            if(std::isnan(tau(i))){
                printf("[ERROR] The setTau function meets Nan\n");
            }
            motorCmd[i].tau = saturation(tau(i), torqueLimit);
        }
    }

    // 设置单腿零速度指令（急停时使用）        
    void setZeroDq(int legID){
        motorCmd[legID*4+0].dq = 0;
        motorCmd[legID*4+1].dq = 0;
        motorCmd[legID*4+2].dq = 0;
        motorCmd[legID*4+3].dq = 0;
    }
    
    // 设置所有腿零速度指令    
    void setZeroDq(){
        for(int i(0); i<4; ++i){
            setZeroDq(i);
        }
    }

    // 设置单腿零力矩指令（被动模式）
    void setZeroTau(int legID){
        motorCmd[legID*4+0].tau = 0;
        motorCmd[legID*4+1].tau = 0;
        motorCmd[legID*4+2].tau = 0;
        motorCmd[legID*4+3].tau = 0;
    }

    // 仿真环境支撑腿参数配置（高刚度）    
    void setSimStanceGain(int legID){
        motorCmd[legID*4+0].mode = 10;
        motorCmd[legID*4+0].Kp = 180;
        motorCmd[legID*4+0].Kd = 8;
        motorCmd[legID*4+1].mode = 10;
        motorCmd[legID*4+1].Kp = 180;
        motorCmd[legID*4+1].Kd = 8;
        motorCmd[legID*4+2].mode = 10;
        motorCmd[legID*4+2].Kp = 300;
        motorCmd[legID*4+2].Kd = 15;
        motorCmd[legID*4+3].mode = 10;
        motorCmd[legID*4+3].Kp = 300;
        motorCmd[legID*4+3].Kd = 15;
    }

    // 真实机器人支撑腿参数（安全刚度）
    void setRealStanceGain(int legID){
        motorCmd[legID*4+0].mode = 10;
        motorCmd[legID*4+0].Kp = 60;
        motorCmd[legID*4+0].Kd = 5;
        motorCmd[legID*4+1].mode = 10;
        motorCmd[legID*4+1].Kp = 40;
        motorCmd[legID*4+1].Kd = 4;
        motorCmd[legID*4+2].mode = 10;
        motorCmd[legID*4+2].Kp = 80;
        motorCmd[legID*4+2].Kd = 7;
        motorCmd[legID*4+3].mode = 10;
        motorCmd[legID*4+3].Kp = 80;
        motorCmd[legID*4+3].Kd = 7;
    }

    // 设置零增益（被动柔顺模式）
    void setZeroGain(int legID){
        motorCmd[legID*4+0].mode = 10;
        motorCmd[legID*4+0].Kp = 0;
        motorCmd[legID*4+0].Kd = 0;
        motorCmd[legID*4+1].mode = 10;
        motorCmd[legID*4+1].Kp = 0;
        motorCmd[legID*4+1].Kd = 0;
        motorCmd[legID*4+2].mode = 10;
        motorCmd[legID*4+2].Kp = 0;
        motorCmd[legID*4+2].Kd = 0;
        motorCmd[legID*4+3].mode = 10;
        motorCmd[legID*4+3].Kp = 0;
        motorCmd[legID*4+3].Kd = 0;
    }

    // 设置所有腿零增益（被动模式）
    void setZeroGain(){
        for(int i(0); i<4; ++i){
            setZeroGain(i);
        }
    }

    // 中等刚度配置（平衡控制用）   
    void setStableGain(int legID){
        motorCmd[legID*4+0].mode = 10;
        motorCmd[legID*4+0].Kp = 0.8;
        motorCmd[legID*4+0].Kd = 0.8;
        motorCmd[legID*4+1].mode = 10;
        motorCmd[legID*4+1].Kp = 0.8;
        motorCmd[legID*4+1].Kd = 0.8;
        motorCmd[legID*4+2].mode = 10;
        motorCmd[legID*4+2].Kp = 0.8;
        motorCmd[legID*4+2].Kd = 0.8;
        motorCmd[legID*4+3].mode = 10;
        motorCmd[legID*4+3].Kp = 0.8;
        motorCmd[legID*4+3].Kd = 0.8;
    }

    // 设置所有腿中等刚度配置（平衡控制）
    void setStableGain(){
        for(int i(0); i<4; ++i){
            setStableGain(i);
        }
    }

    // 摆动腿控制参数（低刚度防止碰撞）    
    void setSwingGain(int legID){
        motorCmd[legID*4+0].mode = 10;
        motorCmd[legID*4+0].Kp = 3;
        motorCmd[legID*4+0].Kd = 2;
        motorCmd[legID*4+1].mode = 10;
        motorCmd[legID*4+1].Kp = 3;
        motorCmd[legID*4+1].Kd = 2;
        motorCmd[legID*4+2].mode = 10;
        motorCmd[legID*4+2].Kp = 3;
        motorCmd[legID*4+2].Kd = 2;
        motorCmd[legID*4+3].mode = 10;
        motorCmd[legID*4+3].Kp = 3;
        motorCmd[legID*4+3].Kd = 2;
    }
};

struct LowlevelCmd{
    MotorCmd motorCmd[12];

    void setQ(Vec12 q){
        for(int i(0); i<12; ++i){
            motorCmd[i].q = q(i);
        }
    }
    void setQ(int legID, Vec3 qi){
        motorCmd[legID*3+0].q = qi(0);
        motorCmd[legID*3+1].q = qi(1);
        motorCmd[legID*3+2].q = qi(2);
    }
    void setQd(Vec12 qd){
        for(int i(0); i<12; ++i){
            motorCmd[i].dq = qd(i);
        }
    }
    void setQd(int legID, Vec3 qdi){
        motorCmd[legID*3+0].dq = qdi(0);
        motorCmd[legID*3+1].dq = qdi(1);
        motorCmd[legID*3+2].dq = qdi(2);
    }
    void setTau(Vec12 tau, Vec2 torqueLimit = Vec2(-50, 50)){
        for(int i(0); i<12; ++i){
            if(std::isnan(tau(i))){
                printf("[ERROR] The setTau function meets Nan\n");
            }
            motorCmd[i].tau = saturation(tau(i), torqueLimit);
        }
    }
    void setZeroDq(int legID){
        motorCmd[legID*3+0].dq = 0;
        motorCmd[legID*3+1].dq = 0;
        motorCmd[legID*3+2].dq = 0;
    }
    void setZeroDq(){
        for(int i(0); i<4; ++i){
            setZeroDq(i);
        }
    }
    void setZeroTau(int legID){
        motorCmd[legID*3+0].tau = 0;
        motorCmd[legID*3+1].tau = 0;
        motorCmd[legID*3+2].tau = 0;
    }
    void setSimStanceGain(int legID){
        motorCmd[legID*3+0].mode = 10;
        motorCmd[legID*3+0].Kp = 180;
        motorCmd[legID*3+0].Kd = 8;
        motorCmd[legID*3+1].mode = 10;
        motorCmd[legID*3+1].Kp = 180;
        motorCmd[legID*3+1].Kd = 8;
        motorCmd[legID*3+2].mode = 10;
        motorCmd[legID*3+2].Kp = 300;
        motorCmd[legID*3+2].Kd = 15;
    }
    void setRealStanceGain(int legID){
        motorCmd[legID*3+0].mode = 10;
        motorCmd[legID*3+0].Kp = 60;
        motorCmd[legID*3+0].Kd = 5;
        motorCmd[legID*3+1].mode = 10;
        motorCmd[legID*3+1].Kp = 40;
        motorCmd[legID*3+1].Kd = 4;
        motorCmd[legID*3+2].mode = 10;
        motorCmd[legID*3+2].Kp = 80;
        motorCmd[legID*3+2].Kd = 7;
    }
    void setZeroGain(int legID){
        motorCmd[legID*3+0].mode = 10;
        motorCmd[legID*3+0].Kp = 0;
        motorCmd[legID*3+0].Kd = 0;
        motorCmd[legID*3+1].mode = 10;
        motorCmd[legID*3+1].Kp = 0;
        motorCmd[legID*3+1].Kd = 0;
        motorCmd[legID*3+2].mode = 10;
        motorCmd[legID*3+2].Kp = 0;
        motorCmd[legID*3+2].Kd = 0;
    }
    void setZeroGain(){
        for(int i(0); i<4; ++i){
            setZeroGain(i);
        }
    }
    void setStableGain(int legID){
        motorCmd[legID*3+0].mode = 10;
        motorCmd[legID*3+0].Kp = 0.8;
        motorCmd[legID*3+0].Kd = 0.8;
        motorCmd[legID*3+1].mode = 10;
        motorCmd[legID*3+1].Kp = 0.8;
        motorCmd[legID*3+1].Kd = 0.8;
        motorCmd[legID*3+2].mode = 10;
        motorCmd[legID*3+2].Kp = 0.8;
        motorCmd[legID*3+2].Kd = 0.8;
    }
    void setStableGain(){
        for(int i(0); i<4; ++i){
            setStableGain(i);
        }
    }
    void setSwingGain(int legID){
        motorCmd[legID*3+0].mode = 10;
        motorCmd[legID*3+0].Kp = 3;
        motorCmd[legID*3+0].Kd = 2;
        motorCmd[legID*3+1].mode = 10;
        motorCmd[legID*3+1].Kp = 3;
        motorCmd[legID*3+1].Kd = 2;
        motorCmd[legID*3+2].mode = 10;
        motorCmd[legID*3+2].Kp = 3;
        motorCmd[legID*3+2].Kd = 2;
    }
};

#endif  //LOWLEVELCMD_H