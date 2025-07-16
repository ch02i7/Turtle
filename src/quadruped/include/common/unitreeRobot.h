
#ifndef UNITREEROBOT_H
#define UNITREEROBOT_H

#include "common/unitreeLeg.h"
#include "message/LowlevelState.h"

// 四足机器人基类（抽象模型）
class QuadrupedRobot{
public:
    QuadrupedRobot(){};  // 默认构造函数
    ~QuadrupedRobot(){}  // 析构函数

    Vec3 getX(LowlevelState &state);          // 获取参考腿（右前腿）在机体坐标系的足端位置
    Vec3 getX_XM(LowlevelState_XM &state);

    Vec34 getVecXP(LowlevelState &state);     // 获取四腿在机体坐标系的足端位置集合（4x3矩阵）
    Vec34 getVecXP_XM(LowlevelState_XM &state); 

    Vec12 getQ(const Vec34 &feetPosition, FrameType frame);                                  // 逆运动学计算（基于机体/髋关节坐标系）
    Vec12 getQd(const Vec34 &feetPosition, const Vec34 &feetVelocity, FrameType frame);      // 逆运动学速度计算（基于机体/髋关节坐标系）
    Vec12 getTau(const Vec12 &q, const Vec34 feetForce);                                     // 关节力矩计算（基于关节角度）

    // Inverse Kinematics(Body/Hip Frame)
    Vec16 getQ_XM(const Vec34 &feetPosition, FrameType frame);                                  // 逆运动学计算（基于机体/髋关节坐标系）
    Vec16 getQd_XM(const Vec34 &feetPosition, const Vec34 &feetVelocity, FrameType frame);      // 逆运动学速度计算（基于机体/髋关节坐标系）
    Vec16 getTau_XM(const Vec16 &q, const Vec34 feetForce);                                     // 关节力矩计算（基于关节角度）

    // Forward Kinematics
    Vec3 getFootPosition_XM(LowlevelState_XM &state, int id, FrameType frame);                     // 获取指定腿在指定坐标系下的足端位置
    Vec3 getFootPosition(LowlevelState &state, int id, FrameType frame);                     // 获取指定腿在指定坐标系下的足端位置
    
    Vec3 getFootVelocity_XM(LowlevelState_XM &state, int id);                                      // 获取指定腿的足端线速度（机体坐标系）
    Vec3 getFootVelocity(LowlevelState &state, int id);         
  
    Vec34 getFeet2BPositions(LowlevelState &state, FrameType frame);                         // 获取所有足端在指定坐标系下的位置
    Vec34 getFeet2BPositions_XM(LowlevelState_XM &state, FrameType frame);                         // 获取所有足端在指定坐标系下的位置
   
    Vec34 getFeet2BVelocities(LowlevelState &state, FrameType frame);                        // 获取所有足端在指定坐标系下的速度
    Vec34 getFeet2BVelocities_XM(LowlevelState_XM &state, FrameType frame);                        // 获取所有足端在指定坐标系下的速度

    Vec3 getBuoyancyCenterPosition(LowlevelState_XM &state, int id, FrameType frame);                     // 获取指定腿在指定坐标系下的浮心位置
    Vec3 getBuoyancyCenterVelocity(LowlevelState_XM &state, int id);                                      // 获取指定腿的浮心线速度（机体坐标系）
    Vec34 getBuoyancyCenter2BPositions(LowlevelState_XM &state, FrameType frame);                         // 获取所有浮心在指定坐标系下的位置
    Vec34 getBuoyancyCenter2BVelocities(LowlevelState_XM &state, FrameType frame);                        // 获取所有浮心在指定坐标系下的速度

    Vec3 getMassCenterPosition(LowlevelState_XM &state, int id, FrameType frame);                          // 获取指定腿在指定坐标系下的质心位置
    Vec3 getMassCenterVelocity(LowlevelState_XM &state, int id);                                           // 获取指定腿的质心线速度（机体坐标系）
    Vec34 getMassCenter2BPositions(LowlevelState_XM &state, FrameType frame);                              // 获取所有质心在指定坐标系下的位置
    Vec34 getMassCenter2BVelocities(LowlevelState_XM &state, FrameType frame);                             // 获取所有质心在指定坐标系下的速度

    Mat3 getJaco(LowlevelState &state, int legID);              // 雅可比矩阵计算
    Mat3 getJaco_XM(LowlevelState_XM &state, int legID);              // 雅可比矩阵计算
    
    Vec2 getRobVelLimitX(){return _robVelLimitX;}               // 获取机器人X轴速度限制
    Vec2 getRobVelLimitY(){return _robVelLimitY;}               // 获取机器人Y轴速度限制
    Vec2 getRobVelLimitYaw(){return _robVelLimitYaw;}           // 获取机器人偏航角速度限制
    Vec34 getFeetPosIdeal(){return _feetPosNormalStand;}        // 获取理想足端位置（站立）
    double getRobMass(){return _mass;}                          // 获取机器人质量
    Vec3 getPcb(){return _pcb;}                                 // 获取机器人重心位置
    Mat3 getRobInertial(){return _Ib;}                          // 获取机器人惯性矩阵

protected:
    QuadrupedLeg* _Legs[4];       // 四腿控制器指针数组
    Vec2 _robVelLimitX;           // X轴速度限制 [min, max]（m/s）
    Vec2 _robVelLimitY;           // Y轴速度限制 [min, max]（m/s）
    Vec2 _robVelLimitYaw;         // 偏航角速度限制 [min, max]（rad/s）
    Vec34 _feetPosNormalStand;    // 正常站立时的四足位置（机体坐标系，4x3矩阵）
    double _mass;                 // 机器人总质量（kg）
    Vec3 _pcb;                    // 质心在机体坐标系中的坐标（m）
    Mat3 _Ib;                     // 机体坐标系下的转动惯量矩阵（kg·m²）
};

// A1型号机器人具体实现
class A1Robot : public QuadrupedRobot{
public:
    A1Robot();  // 构造函数（在源文件中实现具体参数配置）
    ~A1Robot(){}
};

// Go1型号机器人具体实现 
class Go1Robot : public QuadrupedRobot{
public:
    Go1Robot();  // 构造函数（在源文件中实现具体参数配置）
    ~Go1Robot(){};
};

// XMTurtle型号机器人具体实现
class XMTurtle : public QuadrupedRobot{
public:
    XMTurtle();  // 构造函数（在源文件中实现具体参数配置）
    ~XMTurtle(){};
};


#endif  // UNITREEROBOT_H