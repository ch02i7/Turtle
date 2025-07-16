#include "Gait/FeetEndCal.h"
#include "common/unitreeRobot.h"


/**
 * 足端轨迹计算器构造函数
 * @param ctrlComp 控制组件集合（包含状态估计器、机器人模型等）
 * 
 * 初始化流程：
 * 1. 获取步态周期参数（支撑相/摆动相时间）
 * 2. 设置轨迹生成控制增益
 * 3. 计算机器人腿部几何参数（半径和初始角度）
 */
FeetEndCal::FeetEndCal(CtrlComponents *ctrlComp)
           : _est(ctrlComp->estimator),     // 状态估计器
           _lowState(ctrlComp->lowState),   // 底层状态接口
           _robModel(ctrlComp->robotModel), // 机器人模型
           _legModel(ctrlComp->legModel)    // 四足腿运动学模型
            {
    // 从步态生成器获取时间参数
    _Tstance  = ctrlComp->waveGen->getTstance();// 支撑相持续时间（秒）
    _Tswing   = ctrlComp->waveGen->getTswing();// 摆动相持续时间（秒）

    // 轨迹生成控制增益（调节足端轨迹跟踪性能）
    _kx = 0.005;// X轴位置误差增益
    _ky = 0.005;// Y轴位置误差增益
    _kyaw = 0.005;// 偏航角误差增益

    // 获取理想站立姿态下的足端位置（机体坐标系）
    Vec34 feetPosBody = _robModel->getFeetPosIdeal();

    // 计算每条腿的几何参数
    for(int i(0); i<4; ++i){
        _feetRadius(i)    = sqrt( pow(feetPosBody(0, i), 2) + pow(feetPosBody(1, i), 2) );// 足端到机体原点距离
        _feetInitAngle(i) = atan2(feetPosBody(1, i), feetPosBody(0, i));// 足端初始安装角度（相对机体X轴）
    }
}

FeetEndCal::~FeetEndCal(){

}

/**
 * 计算指定腿的足端目标位置
 * @param legID 腿的编号（0-3）
 * @param vxyGoalGlobal 全局坐标系下的目标速度指令 [vx, vy]
 * @param dYawGoal 目标偏航角速度（rad/s）
 * @param phase 当前步态相位（0-1）
 * @return 足端目标位置（全局坐标系）
 * 
 * 实现原理：
 * 1. 基于当前机体速度预测下一周期足端位置
 * 2. 加入速度误差补偿项（PID前馈）
 * 3. 考虑偏航运动引起的足端轨迹偏移
 */
Vec3 FeetEndCal::calFootPos(int legID, Vec2 vxyGoalGlobal, float dYawGoal, float phase){

    // 获取当前机体状态
    _bodyVelGlobal = _est->getVelocity();// 机体线速度（全局坐标系）
    _bodyWGlobal = _lowState->getGyroGlobal();// 机体角速度（全局坐标系）

    // 计算X/Y轴预测步长（基于速度前馈和误差补偿）
    _nextStep(0) = _bodyVelGlobal(0)*(1-phase)*_Tswing// 速度前馈项（摆动相剩余时间）
                 + _bodyVelGlobal(0)*_Tstance/2// 支撑相基础偏移
                 + _kx*(_bodyVelGlobal(0) - vxyGoalGlobal(0));// 速度误差补偿

    _nextStep(1) = _bodyVelGlobal(1)*(1-phase)*_Tswing // 速度前馈项（摆动相剩余时间）
                 + _bodyVelGlobal(1)*_Tstance/2// 支撑相基础偏移
                 + _ky*(_bodyVelGlobal(1) - vxyGoalGlobal(1));// 速度误差补偿

    // Z轴初始化为0，后续处理高度
    _nextStep(2) = 0;

    // 计算偏航角补偿量
    _yaw = _lowState->getYaw();// 当前偏航角
    _dYaw = _lowState->getDYaw();// 当前偏航角速度

    _nextYaw = _dYaw*(1-phase)*_Tswing// 角速度前馈
             + _dYaw*_Tstance/2
             + _kyaw*(dYawGoal - _dYaw);// 角速度误差补偿

     // 添加偏航运动引起的足端位置偏移
    _nextStep(0) += _feetRadius(legID) * cos(_yaw + _feetInitAngle(legID) + _nextYaw);
    _nextStep(1) += _feetRadius(legID) * sin(_yaw + _feetInitAngle(legID) + _nextYaw);

    // 合成最终足端位置（全局坐标系）
    _footPos = _est->getPosition() + _nextStep;// 基于机体位置叠加偏移量
    _footPos(2) = 0.0;// Z轴位置由后续摆动轨迹生成器处理

    return _footPos;
}

Vec3 FeetEndCal::calBuoyancyPos(int legID, Vec2 vxyGoalGlobal, float dYawGoal, float dPitchGoal, float phase){

    // 基础浮心位置（使用机器人模型获取）
    _bodyVelGlobal = _est->getVelocity();                               // 机体线速度（全局坐标系）
    _bodyWGlobal = _lowState->getGyroGlobal();                          // 机体角速度（全局坐标系）

    //获取当前浮心位置
    _footPos =_est->getPosition();                                 // 足端位置（全局坐标系）
    _buoyancyCenter = _legModel->calcBuoyancyCenter(_footPos);     // 浮心位置（全局坐标系）
    
    // 前进速度补偿（相位相关的摆线运动）
    float strokeGain = 0.2f; // 划水幅度增益系数
    float strokeX = vxyGoalGlobal(0) * strokeGain * (1 - cos(2*M_PI*phase));
    
    // 偏航补偿（差动浮心调整）
    float yawGain = 0.5f;    // 偏航补偿系数
    float yawComp = dYawGoal * yawGain;

    // 俯仰补偿（俯仰浮心调整）
    float pitchGain = 0.3f;   // 俯仰补偿系数
    float pitchComp = dPitchGoal * pitchGain;


    _nextStep(0) = strokeX + yawComp;
    _nextStep(1) = 0;
    _nextStep(2) = pitchComp;

    _buoyancyCenter += _nextStep;
      
    return _buoyancyCenter;
}

Vec3 FeetEndCal::calBuoyancyPos_B(int legID, Vec2 vxyGoalGlobal, float dYawGoal, float dPitchGoal, float phase){

    // 基础浮心位置（使用机器人模型获取）
    _bodyVelGlobal = _est->getVelocity();                               // 机体线速度（全局坐标系）
    _bodyWGlobal = _lowState->getGyroGlobal();                          // 机体角速度（全局坐标系）

    //获取当前浮心位置
    _footPos =_est->getPosition();                                 // 足端位置（全局坐标系）
    _buoyancyCenter = _legModel->calcBuoyancyCenter(_footPos);     // 浮心位置（全局坐标系）
    
    // 前进速度补偿（相位相关的摆线运动）
    float strokeGain = 0.2f; // 划水幅度增益系数
    float strokeX = vxyGoalGlobal(0) * strokeGain * (1 - cos(2*M_PI*phase));
    
    // 偏航补偿（差动浮心调整）
    float yawGain = 0.5f;    // 偏航补偿系数
    float yawComp = dYawGoal * yawGain;

    // 俯仰补偿（俯仰浮心调整）
    float pitchGain = 0.3f;   // 俯仰补偿系数
    float pitchComp = dPitchGoal * pitchGain;


    _nextStep(0) = pitchComp;
    _nextStep(1) = 0;
    _nextStep(2) = 0;

    _buoyancyCenter += _nextStep;
      
    return _buoyancyCenter;
}