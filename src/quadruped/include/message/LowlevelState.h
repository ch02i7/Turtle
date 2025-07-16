/**********************************************************************
 底层状态数据结构
 功能：封装从电机驱动器和IMU获取的原始传感器数据
 包含：12个关节状态、IMU数据、用户控制指令
***********************************************************************/
#ifndef LOWLEVELSTATE_HPP
#define LOWLEVELSTATE_HPP

#include <iostream>
#include "common/mathTypes.h"
#include "common/mathTools.h"
#include "interface/CmdPanel.h"
#include "common/enumClass.h"

// 单个电机实时状态
struct MotorState
{
    unsigned int mode;    // 当前控制模式
    float q;              // 实际关节位置 (rad)
    float dq;             // 实际关节速度 (rad/s)
    float ddq;            // 关节加速度 (rad/s²) 
    float tauEst;         // 估计关节力矩 (Nm)

    MotorState() {        // 安全初始化
        q = 0;
        dq = 0;
        ddq = 0;
        tauEst = 0;
    }
};

// IMU传感器数据结构
struct IMU
{
    float quaternion[4];    // 四元数 [w, x, y, z]
    float gyroscope[3];     // 角速度 (rad/s) [x, y, z]
    float accelerometer[3]; // 加速度 (m/s²) [x, y, z]

    IMU(){                 // 初始化归零
        for(int i = 0; i < 3; i++){
            quaternion[i] = 0;
            gyroscope[i] = 0;
            accelerometer[i] = 0;
        }
        quaternion[3] = 0;  // 四元数w分量初始化
    }


    // 获取旋转矩阵（机体坐标系到世界坐标系）
    RotMat getRotMat(){
        Quat quat;
        quat << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
        return quatToRotMat(quat);
    }

    // 获取加速度矢量（机体坐标系）
    Vec3 getAcc(){
        Vec3 acc;
        acc << accelerometer[0], accelerometer[1], accelerometer[2];
        return acc;
    }

    // 获取角速度矢量（机体坐标系）
    Vec3 getGyro(){
        Vec3 gyro;
        gyro << gyroscope[0], gyroscope[1], gyroscope[2];
        return gyro;
    }

    // 获取四元数（用于姿态解算）
    Quat getQuat(){
        Quat q;
        q << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
        return q;
    }
};


// 整机底层状态信息
struct LowlevelState
{
    IMU imu;                    // 惯性测量单元数据
    MotorState motorState[12];  // 16个关节状态（四腿 x 三关节）
    UserCommand userCmd;        // 用户控制指令（遥控器/上位机）
    UserValue userValue;        // 用户自定义参数
    std::string serial_tip ;

    /**
     * 获取所有腿部关节角度（12个关节）
     * @return 3x4矩阵，每列对应一条腿的关节角度 [髋, 膝, 踝] (rad)
     * 
     * 数据结构说明：
     * - 列索引 0-3 分别对应：左前腿、右前腿、左后腿、右后腿
     * - 行索引 0-2 分别对应：髋关节、膝关节、踝关节
     * 
     * 数据来源：
     * 从底层电机状态中实时读取的关节角度信息
     */
    Vec34 getQ(){
        Vec34 qLegs;
        for(int i(0); i < 4; ++i){
            qLegs.col(i)(0) = motorState[3*i    ].dq;
            qLegs.col(i)(1) = motorState[3*i + 1].dq;
            qLegs.col(i)(2) = motorState[3*i + 2].dq;
        }
        return qLegs;
    }

    // 获取所有腿部关节速度（数据结构同getQ）
    Vec34 getQd(){
        Vec34 qdLegs;
        for(int i(0); i < 4; ++i){ 
            qdLegs.col(i)(0) = motorState[3*i    ].dq;
            qdLegs.col(i)(1) = motorState[3*i + 1].dq;
            qdLegs.col(i)(2) = motorState[3*i + 2].dq;
        }
        return qdLegs;
    }

    // 获取当前姿态旋转矩阵（用于坐标变换）
    RotMat getRotMat(){
        return imu.getRotMat();
    }

    // 获取原始加速度（机体坐标系）
    Vec3 getAcc(){
        return imu.getAcc();
    }

    // 获取原始角速度（机体坐标系）
    Vec3 getGyro(){
        return imu.getGyro();
    }

    // 获取全局坐标系加速度（世界坐标系）
    Vec3 getAccGlobal(){
        return getRotMat() * getAcc();
    }

    // 获取全局坐标系角速度（世界坐标系）
    Vec3 getGyroGlobal(){
        return getRotMat() * getGyro();
    }

    // 获取当前偏航角（绕Z轴旋转角度）
    double getYaw(){
        return rotMatToRPY(getRotMat())(2);// 提取Yaw分量
    }

    // 获取偏航角速度（绕Z轴旋转速度）
    double getDYaw(){
        return getGyroGlobal()(2); // Z轴角速度分量
    }

    double getPitch(){
        return rotMatToRPY(getRotMat())(1);// 提取Pitch分量
    }

    double getDPitch(){
        return getGyroGlobal()(1); // Y轴角速度分量
    }

    // 设置模拟关节角度（调试用）
    void setQ(Vec16 q){
        for(int i(0); i<16; ++i){
            motorState[i].q = q(i);
        }
    }
};

struct LowlevelState_XM
{
    IMU imu;                    // 惯性测量单元数据
    MotorState motorState[16];  // 16个关节状态（四腿 x 三关节）
    UserCommand userCmd;        // 用户控制指令（遥控器/上位机）
    UserValue userValue;        // 用户自定义参数
    std::string serial_tip ;



    /**
     * 获取所有腿部关节角度（12个关节）
     * @return 3x4矩阵，每列对应一条腿的关节角度 [髋, 膝, 踝] (rad)
     * 
     * 数据结构说明：
     * - 列索引 0-3 分别对应：左前腿、右前腿、左后腿、右后腿
     * - 行索引 0-2 分别对应：髋关节、膝关节、踝关节
     * 
     * 数据来源：
     * 从底层电机状态中实时读取的关节角度信息
     */
  Mat4 getQ(){
          Mat4 qLegs;
        for(int i(0); i < 4; ++i){
            qLegs.col(i)(0) = motorState[4*i    ].q;// 髋关节
            qLegs.col(i)(1) = motorState[4*i + 1].q;// 膝关节
            qLegs.col(i)(2) = motorState[4*i + 2].q;// 踝关节
            qLegs.col(i)(3) = motorState[4*i + 3].q;// 膝关节
        }
        return qLegs;
    }

    // 获取所有腿部关节速度（数据结构同getQ）
    Mat4 getQd(){
          Mat4 qdLegs;
        for(int i(0); i < 4; ++i){ 
            qdLegs.col(i)(0) = motorState[4*i    ].dq;
            qdLegs.col(i)(1) = motorState[4*i + 1].dq;
            qdLegs.col(i)(2) = motorState[4*i + 2].dq;
            qdLegs.col(i)(3) = motorState[4*i + 3].dq;
        }
        return qdLegs;
    }

    // 获取当前姿态旋转矩阵（用于坐标变换）
    RotMat getRotMat(){
        return imu.getRotMat();
    }

    // 获取原始加速度（机体坐标系）
    Vec3 getAcc(){
        return imu.getAcc();
    }

    // 获取原始角速度（机体坐标系）
    Vec3 getGyro(){
        return imu.getGyro();
    }

    // 获取全局坐标系加速度（世界坐标系）
    Vec3 getAccGlobal(){
        return getRotMat() * getAcc();
    }

    // 获取全局坐标系角速度（世界坐标系）
    Vec3 getGyroGlobal(){
        return getRotMat() * getGyro();
    }

    // 获取当前偏航角（绕Z轴旋转角度）
    double getYaw(){
        return rotMatToRPY(getRotMat())(2);// 提取Yaw分量
    }

    // 获取偏航角速度（绕Z轴旋转速度）
    double getDYaw(){
        return getGyroGlobal()(2); // Z轴角速度分量
    }

    double getPitch(){
        return rotMatToRPY(getRotMat())(1);// 提取Pitch分量
    }

    double getDPitch(){
        return getGyroGlobal()(1); // Y轴角速度分量
    }

    // 设置模拟关节角度（调试用）
    void setQ(Vec16 q){
        for(int i(0); i<16; ++i){
            motorState[i].q = q(i);
        }
    }
};
#endif  //LOWLEVELSTATE_HPP