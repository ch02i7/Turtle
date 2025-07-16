
#ifndef UNITREELEG_H
#define UNITREELEG_H

#include "common/mathTypes.h"
#include "common/enumClass.h"

/**
 * 四足机器人单腿运动学基类
 * 包含正/逆运动学计算、速度计算、雅可比矩阵等核心功能
 */
class QuadrupedLeg{
public:
    /**
     * @brief 腿部构造函数
     * @param legID 腿标识（0-3对应不同腿）
     * @param abadLinkLength 髋关节侧摆连杆长度（米）
     * @param hipLinkLength 髋关节长度（米）
     * @param kneeLinkLength 膝关节长度（米） 
     * @param pHip2B 髋关节在机体坐标系中的安装位置
     */
    QuadrupedLeg(int legID,                     // 腿标识
                float abadLinkLength,           // 髋关节侧摆连杆长度
                float hipLinkLength,            // 髋关节长度
                float kneeLinkLength,           // 膝关节长度
                float BuoyancyCenterLength,     // 浮心长度
                Vec3 BuoyancyQ,                 // 浮心位置
                float MassCenterLength,         // 质心长度
                Vec3 MassQ,                     // 质心位置
                Vec3 pHip2B,                    // 髋关节在机体坐标系中的安装位置
                Vec3 hipAngle                    // 髋关节初始角度
            );
    ~QuadrupedLeg(){}

    /// 正运动学：计算足端在髋关节坐标系中的位置（输入关节角度）    
    Vec3 calcPEe2H(Vec3 q);
    Vec3 calcPEe2H_F(Vec4 q);
    Vec3 calcPEe2H_B(Vec4 q);

    /// 正运动学：计算足端在机体坐标系中的位置    
    Vec3 calcPEe2B(Vec3 q);
    Vec3 calcPEe2B_F(Vec4 q);
    Vec3 calcPEe2B_B(Vec4 q);

    /// 计算足端线速度（基于关节角度和角速度）   
    Vec3 calcVEe(Vec3 q, Vec3 qd);
    Vec3 calcVEe_F(Vec4 q, Vec4 qd);
    Vec3 calcVEe_B(Vec4 q, Vec4 qd);

    /// 逆运动学：计算关节角度（输入足端位置）
    Vec3 calcQ(Vec3 pEe, FrameType frame);
    Vec4 calcQ_F(Vec3 pEe, FrameType frame);    
    Vec4 calcQ_B(Vec3 pEe, FrameType frame);

    /// 逆运动学：计算关节角速度（已知关节角度和足端速度）
    Vec3 calcQd(Vec3 q, Vec3 vEe);
    Vec4 calcQd_F(Vec4 q, Vec3 vEe);
    Vec4 calcQd_B(Vec4 q, Vec3 vEe);

    /// 逆运动学：计算关节角速度（已知足端位置和速度）
    Vec3 calcQd(Vec3 pEe, Vec3 vEe, FrameType frame);
    Vec4 calcQd_F(Vec3 pEe, Vec3 vEe, FrameType frame);
    Vec4 calcQd_B(Vec3 pEe, Vec3 vEe, FrameType frame);

    /// 计算关节力矩（输入关节角度和力）
    Vec3 calcTau(Vec3 q, Vec3 force);
    Vec4 calcTau_F(Vec4 q, Vec3 force);
    Vec4 calcTau_B(Vec4 q, Vec3 force);

    /// 雅可比矩阵计算（输入关节角度）
    Mat3 calcJaco(Vec3 q);
    Mat3 calcJaco_F(Vec4 q);
    Mat3 calcJaco_B(Vec4 q);

    //质心位置获取
    Vec3 calcPEe2H_Mass(Vec3 q);
    Vec3 calcPEe2B_Mass(Vec3 q);

    //浮心位置获取
    Vec3 calcPEe2H_Buoyancy(Vec3 q);
    Vec3 calcPEe2B_Buoyancy(Vec3 q);

    // 浮心质心动力学计算方法
    Vec3 calcVEe_Mass(Vec3 q, Vec3 qd);    // 质心速度计算
    Vec3 calcVEe_Buoyancy(Vec3 q, Vec3 qd); // 浮心速度计算
    Vec3 calcTau_Mass(Vec3 q, Vec3 force);  // 质心力矩映射
    Vec3 calcTau_Buoyancy(Vec3 q, Vec3 force); // 浮心力矩映射


    /// 辅助函数：获取髋关节安装位置
    Vec3 getHip2B(){return _pHip2B;}
    Vec3 getBuoyancyQ(){return _BuoyancyQ;}    
    Vec3 getMassQ(){return _MassQ;}

    Vec3 calcMassCenter(Vec3 pEe);
    Vec3 calcBuoyancyCenter(Vec3 pEe);  

protected:
    float q1_ik(float py, float pz, float b2y);  ///< 计算髋关节侧摆角q1
    float q3_ik(float b3z, float b4z, float b);   ///< 计算膝关节角度q3
    float q2_ik(float q1, float q3, float px,     ///< 计算髋关节俯仰角q2
                float py, float pz, float b3z, float b4z);

    float _sideSign;  ///< 腿侧标志（-1为左侧，1为右侧）
    const float _abadLinkLength, _hipLinkLength, _kneeLinkLength;  ///< 机械参数
    const float _BuoyancyCenterLength, _MassCenterLength;  ///< 机械参数
    const Vec3 _pHip2B;  ///< 髋关节安装位置
    const Vec3 _hipAngle;  ///< 髋关节初始角度
    const Vec3 _MassQ;  
    const Vec3 _BuoyancyQ;  
};

class A1Leg : public QuadrupedLeg{
public:
    A1Leg(const int legID, 
            const Vec3 pHip2B, 
            const Vec3 hipAngle,
            const Vec3 MassQ , 
            const Vec3 BuoyancyQ) : 
        QuadrupedLeg(legID, 0.08, 0.213, 0.213, 0.213, BuoyancyQ, 0.213, MassQ, pHip2B, hipAngle) {}
    ~A1Leg(){}
};

class Go1Leg : public QuadrupedLeg{
public:
    Go1Leg(const int legID, 
            const Vec3 pHip2B, 
            const Vec3 hipAngle,
            const Vec3 MassQ , 
            const Vec3 BuoyancyQ) : 
        QuadrupedLeg(legID, 0.08, 0.213, 0.213, 0.213, BuoyancyQ, 0.213, MassQ, pHip2B, hipAngle) {}
    ~Go1Leg(){}
};

class XMturtle : public QuadrupedLeg {  
public:
    XMturtle(const int legID,
            const Vec3 pHip2B, 
            const Vec3 hipAngle,
            const Vec3 MassQ , 
            const Vec3 BuoyancyQ) : 
        QuadrupedLeg(legID, 0.63, 1.378, 0.0, 0.213, BuoyancyQ, 0.213, MassQ, pHip2B, hipAngle) {}

    ~XMturtle() {}
};


#endif  // UNITREELEG_H