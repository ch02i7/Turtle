
#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <vector>
#include "common/unitreeRobot.h"    // 机器人模型定义
#include "common/LowPassFilter.h"   // 低通滤波器实现
#include "Gait/WaveGenerator.h"     // 步态生成器
#include "message/LowlevelState.h"  // 底层状态消息定义
#include "string"                   // 字符串处理

#ifdef COMPILE_DEBUG
    #include "common/PyPlot.h"
#endif  // COMPILE_DEBUG

#ifdef COMPILE_WITH_MOVE_BASE
    #include <ros/ros.h>
    #include <ros/time.h>
    #include <geometry_msgs/TransformStamped.h>
    #include <tf/transform_broadcaster.h>
    #include <nav_msgs/Odometry.h>
    #include <geometry_msgs/Twist.h>
    #include <boost/array.hpp>
#endif  // COMPILE_WITH_MOVE_BASE


/**
 * 四足机器人状态估计器
 * 功能：融合IMU、关节编码器和足端接触信息，通过卡尔曼滤波估计机器人本体状态
 * 包含：
 * - 本体位置/速度估计
 * - 足端运动学计算
 * - ROS里程计发布
 */
class Estimator{
public:
    // 构造函数（完整参数版和简化版）
    Estimator(QuadrupedRobot *robotModel, LowlevelState* lowState, VecInt4 *contact, Vec4 *phase, double dt);

    Estimator(QuadrupedRobot *robotModel, LowlevelState* lowState, VecInt4 *contact, Vec4 *phase, double dt, Vec18 Qdig, std::string testName);
    ~Estimator();

    // 状态获取接口  
    Vec3  getPosition();                // 获取全局坐标系下的位置估计
    Vec3  getVelocity();                // 获取全局坐标系下的速度估计
    
    Vec3  getFootPos(int i);            // 获取指定足端全局坐标
    Vec34 getFeetPos();                 // 获取全部足端全局坐标
    Vec34 getFeetVel();                 // 获取全部足端全局速度
    Vec34 getPosFeet2BGlobal();         // 获取足端相对本体位置

    Vec3 getBuoyancyCenterPos(int i);  // 获取指定浮心全局坐标
    Vec34 getBuoyancyCenterPos();       // 获取全部足端全局坐标
    Vec34 getBuoyancyCenterVel(int i);  // 获取指定浮心全局速度
    Vec34 getBuoyancyCenterVel();       // 获取全部浮心全局速度
    Vec34 getPosFeet2BGlobalBuoyancyCenter();         // 获取浮心相对本体位置

    Vec3 getMassCenterPos(int i);      // 获取指定质心全局坐标
    Vec34 getMassCenterPos();           // 获取全部质心全局坐标
    Vec34 getMassCenterVel(int i);      // 获取指定质心全局速度
    Vec34 getMassCenterVel();           // 获取全部质心全局速度
    Vec34 getPosFeet2BGlobalMassCenter();     // 获取质心相对本体位置

    void run();                 // 主估计函数

#ifdef COMPILE_DEBUG
    void setPyPlot(PyPlot *plot){_testPlot = plot;}
#endif  // COMPILE_DEBUG

private:
    void _initSystem();  // 系统初始化

    // 状态空间模型
    Eigen::Matrix<double, 18, 1>  _xhat;    // 状态向量 [位置(3)+速度(3)+足端位置(12)]
    Vec3 _u;                                // 输入向量（加速度）
    Eigen::Matrix<double, 28,  1> _y;       // 观测向量（足端位置/速度测量值）
    Eigen::Matrix<double, 28,  1> _yhat;    // 观测预测值
    Eigen::Matrix<double, 18, 18> _A;       // 状态转移矩阵
    Eigen::Matrix<double, 18, 3>  _B;       // 控制输入矩阵
    Eigen::Matrix<double, 28, 18> _C;       // 观测矩阵

    // 协方差矩阵
    Eigen::Matrix<double, 18, 18> _P;       // 状态协方差
    Eigen::Matrix<double, 18, 18> _Ppriori; // 先验协方差
    Eigen::Matrix<double, 18, 18> _Q;       // 过程噪声协方差
    Eigen::Matrix<double, 28, 28> _R;       // 观测噪声协方差
    Eigen::Matrix<double, 18, 18> _QInit;   // 初始过程噪声协方差
    Eigen::Matrix<double, 28, 28> _RInit;   // 初始观测噪声协方差
    Vec18 _Qdig;                            // 过程噪声对角线参数
    Mat3 _Cu;                               // 输入协方差矩阵

    // 运动学数据
    Eigen::Matrix<double, 12, 1>  _feetPos2Body; // 足端相对本体位置（全局坐标系）
    Eigen::Matrix<double, 12, 1>  _feetVel2Body; // 足端相对本体速度（全局坐标系）
    Eigen::Matrix<double, 4, 1>    _feetH;        // 足端高度测量值

    // 卡尔曼滤波中间矩阵
    Eigen::Matrix<double, 28, 28> _S;               // 新息协方差矩阵 S = C*P_prior*C^T + R
    Eigen::PartialPivLU<Eigen::Matrix<double, 28, 28>> _Slu;    // S矩阵的LU分解，用于高效求解线性方程组
    Eigen::Matrix<double, 28,  1> _Sy;              // 卡尔曼增益中间量：Sy = S⁻¹*(y - y_hat)
    Eigen::Matrix<double, 28, 18> _Sc;              // 卡尔曼增益计算中间量：Sc = S⁻¹*C
    Eigen::Matrix<double, 28, 28> _SR;              // 观测噪声加权项：SR = S⁻¹*R
    Eigen::Matrix<double, 28, 18> _STC;             // 转置矩阵运算中间量：STC = (S^T)⁻¹*C
    Eigen::Matrix<double, 18, 18> _IKC;             // 协方差更新矩阵：IKC = I - K*C (Joseph形式稳定协方差更新

    RotMat _rotMatB2G;                              // 本体到全局的旋转矩阵
    Vec3 _g;                                        // 重力加速度向量
    Vec34 _feetPosGlobalKine, _feetVelGlobalKine;   // 足端位置/速度（全局坐标系）

    // 系统参考
    LowlevelState* _lowState;    // 底层传感器数据指针
    LowlevelState_XM* _lowState_xm;    // 底层传感器数据指针
    QuadrupedRobot *_robModel;   // 机器人运动学模型
    Vec4 *_phase;               // 步态相位指针
    VecInt4 *_contact;          // 足端接触状态指针
    double _dt;                 // 控制周期
    double _trust;              // 相位信任系数
    double _largeVariance;      // 大方差值（用于噪声调整）

    // 信号处理
    LPFilter *_vxFilter, *_vyFilter, *_vzFilter; // 三轴速度低通滤波器

    // 调试工具
    AvgCov *_RCheck;    // 观测噪声监测器
    AvgCov *_uCheck;    // 输入噪声监测器
    std::string _estName; // 估计器名称标识

#ifdef COMPILE_DEBUG
    PyPlot *_testPlot;
#endif  // COMPILE_DEBUG
#ifdef COMPILE_WITH_MOVE_BASE
    ros::NodeHandle _nh;
    ros::Publisher _pub;
    tf::TransformBroadcaster _odomBroadcaster;
    ros::Time _currentTime;
    geometry_msgs::TransformStamped _odomTF;
    nav_msgs::Odometry _odomMsg;
    int _count = 0;
    double _pubFreq = 10;

    Vec3 _velBody, _wBody;
    boost::array<double, 36> _odom_pose_covariance = {1e-9, 0, 0, 0, 0, 0, 
                                        0, 1e-3, 1e-9, 0, 0, 0, 
                                        0, 0, 1e6, 0, 0, 0,
                                        0, 0, 0, 1e6, 0, 0, 
                                        0, 0, 0, 0, 1e6, 0, 
                                        0, 0, 0, 0, 0, 1e-9};
    boost::array<double, 36> _odom_twist_covariance = {1e-9, 0, 0, 0, 0, 0, 
                                        0, 1e-3, 1e-9, 0, 0, 0, 
                                        0, 0, 1e6, 0, 0, 0, 
                                        0, 0, 0, 1e6, 0, 0, 
                                        0, 0, 0, 0, 1e6, 0, 
                                        0, 0, 0, 0, 0, 1e-9};
#endif  // COMPILE_WITH_MOVE_BASE

};

#endif  // ESTIMATOR_H