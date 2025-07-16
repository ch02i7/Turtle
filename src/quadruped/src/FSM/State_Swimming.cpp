#include "FSM/State_Swimming.h"
#include <cmath>
#include <iomanip>


State_Swimming::State_Swimming(CtrlComponents *ctrlComp)
            : FSMState(ctrlComp, FSMStateName::SWIMMING, "swimming"),
                _est(ctrlComp->estimator),                          // 状态估计器
                _phase(ctrlComp->phase),                            // 步态相位生成器               
                _robModel(ctrlComp->robotModel)                     // 机器人运动学模型

{
    _gait = new GaitGenerator(ctrlComp);        // 步态轨迹生成器
    ctrlComp->waveGen = new WaveGenerator(0.45, 0.5, Vec4(0, 0.5, 0.5, 0)); // 初始化步态生成器


    _KpSwing = Vec3(300, 300, 300).asDiagonal(); // 降低摆动刚度
    _KdSwing = Vec3(20, 20, 20).asDiagonal();   // 增加速度阻尼

    _vxLim = _robModel->getRobVelLimitX();// X轴速度限制
    _vyLim = _robModel->getRobVelLimitY();// Y轴速度限制
    _wyawLim = _robModel->getRobVelLimitYaw();// 偏航角速度限制
}


State_Swimming::~State_Swimming(){
    delete _gait;
}

/**
 * 进入游泳状态初始化：
 * 1. 重置期望位置为当前位置
 * 2. 初始化速度指令
 * 3. 重置步态生成器
 */
void State_Swimming::enter(){
    _pcd = _est->getPosition();// 获取当前机体位置

    _vCmdBody.setZero();// 速度指令清零

    _yawCmd = _lowState->getYaw();// 获取当前偏航角
    _Rd = rotz(_yawCmd);// 初始化旋转矩阵

    _wCmdGlobal.setZero();// 角速度指令清零
    _ctrlComp->ioInter->zeroCmdPanel();// 清空控制面板
    _gait->restart();// 重启步态生成
}

void State_Swimming::exit(){
    _ctrlComp->ioInter->zeroCmdPanel();// 清空IO控制面板指令
    _ctrlComp->setAllSwing();// 设置所有腿部为摆动模式（停止力控）
}

/**
 * 退出游泳状态时的清理操作
 * 功能：
 * 1. 清空控制面板指令
 * 2. 设置所有关节为摆动模式（释放电机力矩控制）
 * 
 * 触发场景：
 * - 状态机切换到其他状态前
 * - 紧急停止时
 */
FSMStateName State_Swimming::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;// 遥控器L2+B组合键触发
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;// 遥控器L2+A组合键触发
    }
    else{
        return FSMStateName::SWIMMING;// 默认保持游泳状态
    }
}

void State_Swimming::run(){
    
    // 获取状态估计数据
    _posBody = _est->getPosition();// 机体世界坐标系位置（X-Y-Z）
    _velBody = _est->getVelocity();// 机体线速度（全局坐标系）

    _posFeet2BGlobal = _est->getPosFeet2BGlobalBuoyancyCenter(); // 足端位置（全局坐标系）
    _posFeetGlobal = _est->getBuoyancyCenterPos();// 足端位置（全局坐标系）
    _velFeetGlobal = _est->getBuoyancyCenterVel();// 足端速度（全局坐标系）

    _B2G_RotMat = _lowState->getRotMat();// 机体到世界的旋转矩阵
    _G2B_RotMat = _B2G_RotMat.transpose();// 世界到机体的旋转矩阵

    _yaw = _lowState->getYaw();// 机体偏航角
    _dYaw = _lowState->getDYaw();// 机体偏航角速度

    _pitch = _lowState->getPitch();// 机体俯仰角
    _dpitch = _lowState->getDPitch();// 机体俯仰角速度

    _userValue = _lowState->userValue;// 遥控器输入值（LX, LY, RX, RY）

    // 处理用户摇杆输入
    getUserCmd();// 将摇杆值转换为速度指令

    // 计算全局坐标系下的运动指令
    calcCmd();// 包含速度限幅处理

    // 生成步态轨迹（贝塞尔曲线规划）
    _gait->setGait_swimming(_vCmdGlobal.segment(0,2), // 速度指令（X-Y）
                   _wCmdGlobal(2), // 偏航角速度
                   _wCmdGlobal(1));// 俯仰角速度

    _gait->run_swimming_F(_posFeetGlobalGoal, // 足端目标位置
               _velFeetGlobalGoal);// 足端目标速度

    _gait->run_swimming_B(_posFeetGlobalGoal, // 足端目标位置
               _velFeetGlobalGoal);// 足端目标速度

    // 计算关节力矩和期望角度    
    _tau = Eigen::VectorXd::Constant(16, 0.5);
    calcQQd();// 逆运动学解算

    // 设置底层指令
    _lowCmd_XM->setTau(_tau);// 关节力矩
    _lowCmd_XM->setQ(vec44ToVec16(_qGoal));// 关节目标角度
    _lowCmd_XM->setQd(vec44ToVec16(_qdGoal));// 关节目标角速度
}


/**
 * 处理用户遥控器输入生成速度指令
 * 输入处理流程：
 * 1. 运动指令处理：
 *    - 左摇杆垂直轴(ly) → 机体X轴速度（前进/后退）
 *    - 左摇杆水平轴(lx) → 机体Y轴速度（横向移动）
 * 2. 转向指令处理：
 *    - 右摇杆水平轴(rx) → 偏航角速度（左/右转向）
 * 
 * 关键处理：
 * - invNormalize：将[-1,1]摇杆值映射到速度限制范围
 * - 低通滤波：对转向指令进行平滑处理（系数0.9）
 * - 坐标系定义：X向前，Y向左，符合机器人坐标系标准
 */
void State_Swimming::getUserCmd(){
    /* 运动指令处理 */
    _vCmdBody(0) =  invNormalize(_userValue.ly, _vxLim(0), _vxLim(1));// 前向速度
    _vCmdBody(1);
  //_vCmdBody(1) = -invNormalize(_userValue.lx, _vyLim(0), _vyLim(1));// 横向速度（符号取反匹配坐标系）
    _vCmdBody(2) = 0;// Z轴速度保持为0

     /* 转向指令处理 */
    _dYawCmd = -invNormalize(_userValue.lx, _wyawLim(0), _wyawLim(1)); // 原始转向指令
    _dYawCmd = 0.9*_dYawCmdPast + (1-0.9) * _dYawCmd;// 一阶低通滤波（时间常数约10个控制周期）
    _dYawCmdPast = _dYawCmd;// 保存当前指令供下次滤波使用

     /* 俯仰指令处理 */
    _pitchCmd = -invNormalize(_userValue.lx, _wyawLim(0), _wyawLim(1)); // 原始转向指令
    _pitchCmd = 0.9*_dpitchCmdPast + (1-0.9) * _dpitchCmd;// 一阶低通滤波（时间常数约10个控制周期）
    _dpitchCmdPast = _dpitchCmd;// 保存当前指令供下次滤波使用   



}

/**
 * 运动指令计算与处理（坐标转换 + 限幅）
 * 功能流程：
 * 1. 将机体坐标系速度指令转换到全局坐标系
 * 2. 对全局速度进行动态限幅（基于当前实际速度）
 * 3. 积分计算期望位置（带位置约束）
 * 4. 处理偏航角指令（积分计算当前期望偏航角）
 */
void State_Swimming::calcCmd(){
    /* 平移运动处理 */
    _vCmdGlobal = _B2G_RotMat * _vCmdBody; // 机体坐标系→全局坐标系

    // 速度限幅（防止指令突变，限制在±0.2m/s范围内）
    _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_velBody(0)-0.2, _velBody(0)+0.2));
    _vCmdGlobal(1) = 0;

    // 位置积分（带±5cm约束的期望位置更新）
    _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * _ctrlComp->dt, Vec2(_posBody(0) - 0.05, _posBody(0) + 0.05));
    _pcd(1) = 0;
    _vCmdGlobal(2) = 0;// Z轴速度保持为0

    /* 旋转运动处理 */
    _yawCmd = _yawCmd + _dYawCmd * _ctrlComp->dt;// 积分得到当前期望偏航角
    _pitchCmd = _pitchCmd + _dpitchCmd * _ctrlComp->dt;// 积分得到当前期望俯仰角

    _Rd = rotz(_yawCmd);// 生成目标旋转矩阵

    _wCmdGlobal(0) = 0;             // 设置全局坐标系X轴角速度指令
    _wCmdGlobal(1) = _pitchCmd;     // 设置全局坐标系Y轴角速度指令
    _wCmdGlobal(2) = _dYawCmd;      // 设置全局坐标系Z轴角速度指令
}

/**
 * 关节力矩计算（基于全身动力学）
 * 实现流程：
 * 1. 计算位置和速度跟踪误差
 * 2. 通过PD控制生成期望加速度/角加速度
 * 3. 调用平衡控制器计算足端接触力
 * 4. 处理摆动腿的虚拟弹簧阻尼力
 * 5. 转换坐标系并计算机体逆动力学
 */


/**
 * 计算关节目标角度和速度（逆运动学解算）
 * 实现流程：
 * 1. 坐标系转换：将全局坐标系下的足端轨迹转换为机体坐标系
 * 2. 运动学解算：通过机器人模型计算关节角度/速度
 */
void State_Swimming::calcQQd(){


    Vec34 _posFeet2B;
    _posFeet2B = _robModel->getFeet2BPositions(*_lowState,FrameType::BODY);
    
    // 遍历四条腿进行坐标转换
    for(int i(0); i<4; ++i){
        // 全局坐标系→机体坐标系的位置转换
        _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _posBody);
        // 简化版速度转换（未考虑机体旋转带来的附加速度）
        _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody); 
        // 完整版速度转换公式（参考机器人动力学教材公式6.12，当前未启用）
        // _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody - _B2G_RotMat * (skew(_lowState->getGyro()) * _posFeet2B.col(i)) );  //  c.f formula (6.12) 
    }
    
    // 逆运动学解算得到关节目标角度
    _qGoal = vec16ToVec44(_robModel->getQ_XM(_posFeet2BGoal, FrameType::BODY));
    // 逆运动学解算得到关节目标速度
    _qdGoal = vec16ToVec44(_robModel->getQd_XM(_posFeet2B, _velFeet2BGoal, FrameType::BODY));
}

