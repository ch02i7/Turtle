
#ifndef SWIMMING_H
#define SWIMMING_H

#include "FSM/FSMState.h"
#include "Gait/GaitGenerator.h"
#include "control/BalanceCtrl.h"
#include "FSM/FSMState.h"
#include "common/LowPassFilter.h"

class State_Swimming : public FSMState {
public:

    /**
     * @brief 构造函数
     * @param ctrlComp 控制器组件指针
     */
    State_Swimming(CtrlComponents* ctrlComp);
    virtual ~State_Swimming(); // 添加默认析构函数

    // 状态机标准方法
    void enter();// 进入状态初始化
    void run();// 主控制循环（1000Hz执行）
    void exit();// 退出状态清理

    /**
     * 状态切换检测
     * @return 目标状态枚举值
     */
    FSMStateName checkChange();

    /**
     * 设置高层运动指令（外部接口）
     * @param vx 前向速度（m/s）
     * @param vy 横向速度（m/s） 
     * @param wz 偏航角速度（rad/s）
     */
    void setHighCmd(double vx, double vy, double wz);

private:
    // 核心控制方法
    void calcTau();// 关节力矩计算（全身动力学）
    void calcQQd(); // 关节角度/速度解算（逆运动学）
    void calcCmd();// 运动指令处理（坐标转换+限幅）
    virtual void getUserCmd();// 用户指令处理（摇杆信号解析）
    void calcBalanceKp(); // 平衡控制增益计算（基于步态）
    bool checkStepOrNot();// 步态相位切换判断

    // 控制组件
    GaitGenerator *_gait;// 步态生成器实例
    Estimator *_est;// 状态估计器
    QuadrupedRobot *_robModel;// 机器人运动学/动力学模型
    BalanceCtrl *_balCtrl;// 平衡控制器

    // 机器人实时状态
    Vec3  _posBody;     // 机体位置（世界坐标系，单位：m）
    Vec3  _velBody;     // 机体线速度（世界坐标系，单位：m/s）

    double _yaw;        // 当前偏航角（单位：rad）
    double _dYaw;       // 偏航角速度（单位：rad/s）

    double _pitch;      // 当前俯仰角（单位：rad）
    double _dpitch;     // 俯仰角速度（单位：rad/s）

    Vec34 _posFeetGlobal; // 足端全局坐标（4x3矩阵，单位：m）
    Vec34 _velFeetGlobal; // 足端速度（全局坐标系，单位：m/s）
    Vec34 _posFeet2BGlobal; // 足端相对机体坐标（4x3矩阵）
    RotMat _B2G_RotMat; // 机体到世界旋转矩阵
    RotMat _G2B_RotMat; // 世界到机体旋转矩阵
    Vec12 _q;           // 当前关节角度（12维向量）

    // LPFilter _rollFilter;   // 横滚角滤波器
    // LPFilter _pitchFilter;  // 俯仰角滤波器 
    // LPFilter _yawFilter;    // 偏航角滤波器

    // 运动控制指令
    Vec3 _pcd;          // 指令机体位置（世界坐标系）
    Vec3 _vCmdGlobal;   // 全局坐标系速度指令
    Vec3 _vCmdBody;     // 机体坐标系速度指令

    double _yawCmd;     // 目标偏航角
    double _dYawCmd;    // 目标偏航角速度
    double _dYawCmdPast;// 上一次偏航角速度

    double _pitchCmd;   // 目标俯仰角
    double _dpitchCmd;  // 目标俯仰角速度
    double _dpitchCmdPast;// 上一次俯仰角速度

    Vec3 _wCmdGlobal;   // 全局坐标系角速度指令
    Vec34 _posFeetGlobalGoal; // 足端目标位置（全局坐标系）
    Vec34 _velFeetGlobalGoal; // 足端目标速度（全局坐标系）
    Vec34 _posFeet2BGoal;// 足端目标位置（机体坐标系）
    Vec34 _velFeet2BGoal;// 足端目标速度（机体坐标系）
    RotMat _Rd;         // 目标旋转矩阵
    
    // 中间计算结果
    Vec3 _ddPcd;        // 期望线加速度
    Vec3 _dWbd;         // 期望角加速度
    Vec34 _forceFeetGlobal; // 足端接触力（全局坐标系）
    Vec34 _forceFeetBody;  // 足端接触力（机体坐标系）
    Mat4 _qGoal;       // 目标关节角度
    Mat4 _qdGoal;      // 目标关节角速度
    Vec16 _tau;         // 计算得到的关节力矩

    // 控制参数
    double _gaitHeight; // 摆动腿抬升高度（单位：m）
    Vec3 _posError;     // 位置跟踪误差
    Vec3 _velError;     // 速度跟踪误差
    Mat3 _Kpp;         // 位置环比例增益矩阵
    Mat3 _Kdp;         // 速度环微分增益矩阵
    Mat3 _Kdw;         // 旋转微分增益矩阵
    double _kpw;        // 旋转比例增益
    Mat3 _KpSwing;     // 摆动腿位置增益
    Mat3 _KdSwing;     // 摆动腿速度增益
    Vec2 _vxLim;       // X轴速度限制 [min, max]
    Vec2 _vyLim;       // Y轴速度限制
    Vec2 _wyawLim;     // 偏航角速度限制
    Vec4 *_phase;      // 步态相位指针
    VecInt4 *_contact; // 足端接触状态指针

    // 调试信息
    AvgCov *_avg_posError = new AvgCov(3, "_posError", true, 1000, 1000, 1);
    AvgCov *_avg_angError = new AvgCov(3, "_angError", true, 1000, 1000, 1000);
};

#endif  // SWIMMING_H