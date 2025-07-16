#include "common/unitreeLeg.h"
#include <iostream>
#include <common/mathTools.h>



/**
 * 四足单腿模型构造函数
 * @param legID 腿标识 (0:左前, 1:右前, 2:左后, 3:右后)
 * @param abadLinkLength 髋外展/内收连杆长度 (m)
 * @param hipLinkLength 髋屈曲连杆长度 (m)
 * @param kneeLinkLength 膝关节连杆长度 (m)
 * @param pHip2B 髋关节相对于机体基座的安装位置 (三维向量)
 * 
 * 功能说明：
 * 1. 初始化机械参数并验证腿ID有效性
 * 2. 设置左右腿符号标志（用于运动学计算中的方向处理）
 */
QuadrupedLeg::QuadrupedLeg(int legID, float abadLinkLength, float hipLinkLength, 
                           float kneeLinkLength, float BuoyancyCenterLength, 
                           Vec3 BuoyancyQ, float MassCenterLength, Vec3 MassQ, Vec3 pHip2B, Vec3 hipAngle )
            :_abadLinkLength(abadLinkLength),               // 初始化第一关节连杆长度
             _hipLinkLength(hipLinkLength),                 // 初始化第二关节连杆长度
             _kneeLinkLength(kneeLinkLength),               // 初始化龟鳍长度
             _BuoyancyCenterLength(BuoyancyCenterLength),   // 初始化浮心长度
             _BuoyancyQ(BuoyancyQ),                         // 初始化浮心安装位置
             _MassCenterLength(MassCenterLength),           // 初始化质心长度
             _MassQ(MassQ),                                 // 初始化质心安装位置
             _pHip2B(pHip2B),                               // 初始化髋关节安装位置
             _hipAngle(hipAngle)                            // 初始化髋关节角度
            {                              


    // 设置腿部左右侧符号（影响运动学计算方向）
    if (legID == 0 || legID == 2)
        _sideSign = 1;  // 左侧腿符号为负
    else if (legID == 1 || legID == 3)
        _sideSign = -1;   // 右侧腿符号为正
    else{
        std::cout << "Leg ID incorrect!" << std::endl;
        exit(-1); // 严重错误立即终止程序
    }
}

/**
 * 正运动学解算（髋关节坐标系）
 * @param q 关节角度向量 [外展角, 髋屈曲, 膝屈曲] (rad)
 * @return 足端在髋坐标系的三维位置 [x, y, z]^T (m)
 * 
 * 实现原理：
 * 1. 构建三维空间运动链式变换矩阵
 * 2. 通过几何法推导闭合解
 * 
 * 公式推导说明：
 * X轴（前后方向）：由髋屈曲和膝屈曲共同驱动
 * Y轴（侧向方向）：受外展角和复合旋转影响
 * Z轴（垂直方向）：包含侧摆机构和上下连杆的综合作用
 */
Vec3 QuadrupedLeg::calcPEe2H(Vec3 q){
    // 符号化处理后的机构参数
    float l1 = _sideSign * _abadLinkLength; // 侧摆机构有效长度（含左右符号）
    float l2 = -_hipLinkLength;              // 大腿长度（方向补偿）
    float l3 = -_kneeLinkLength;             // 小腿长度（方向补偿）

    // 关节三角函数预计算
    float s1 = std::sin(q(0));  // 外展角正弦
    float s2 = std::sin(q(1));  // 髋屈曲正弦
    float s3 = std::sin(q(2));  // 膝屈曲正弦
    float c1 = std::cos(q(0));  // 外展角余弦
    float c2 = std::cos(q(1));  // 髋屈曲余弦 
    float c3 = std::cos(q(2));  // 膝屈曲余弦

    // 关节角度组合项（q2+q3）
    float c23 = c2 * c3 - s2 * s3;  // cos(q2+q3)
    float s23 = s2 * c3 + c2 * s3;  // sin(q2+q3)

    Vec3 pEe2H;
    // X轴分量：前后方向运动合成
    pEe2H(0) = l3 * s23 + l2 * s2;  // 大腿与小腿的投影合成
    
    // Y轴分量：侧向运动合成（含外展机构影响）
    pEe2H(1) = -l3 * s1 * c23       // 小腿侧向分量
               + l1 * c1            // 侧摆机构本体偏移
               - l2 * c2 * s1;       // 大腿侧向分量
    
    // Z轴分量：垂直方向运动合成
    pEe2H(2) = l3 * c1 * c23       // 小腿垂直分量
               + l1 * s1            // 侧摆机构垂直偏移
               + l2 * c1 * c2;      // 大腿垂直分量

    return pEe2H;
}

Vec3 QuadrupedLeg::calcPEe2H_F(Vec4 q){
    // 符号化处理后的机构参数
    float l1 = _sideSign * _abadLinkLength; // 侧摆机构有效长度（含左右符号）
    float l2 = _hipLinkLength;              // 大腿长度（方向补偿）
    float l3 = _kneeLinkLength;             // 小腿长度（方向补偿）

    // 关节三角函数预计算
    float s1 = std::sin(q(0));  // 外展角正弦
    float s2 = std::sin(q(1));  // 髋屈曲正弦
    float s3 = std::sin(q(2));  // 膝屈曲正弦
    float c1 = std::cos(q(0));  // 外展角余弦
    float c2 = std::cos(q(1));  // 髋屈曲余弦 
    float c3 = std::cos(q(2));  // 膝屈曲余弦

    // 关节角度组合项（q2+q3）
    float c23 = c2 * c3 - s2 * s3;  // cos(q2+q3)
    float s23 = s2 * c3 + c2 * s3;  // sin(q2+q3)

    float t = l3 * sin(M_PI * 30 / 180) + 15 * 180 * q(1) / M_PI; // 简化物理意义明确的表达式
    float st = sin(M_PI * 30 / 180); // 使用标准π值
    float ct = cos(M_PI * 30 / 180);

    Vec3 pEe2H;
    // X轴分量：前后方向运动合成
    pEe2H(0) = l1 * c1 + t * st * c1 ;  // 大腿与小腿的投影合成
    
    // Y轴分量：侧向运动合成（含外展机构影响）
    pEe2H(1) = l1 * s1 + t * st * s1 ;       // 小腿侧向分量
    
    // Z轴分量：垂直方向运动合成
    pEe2H(2) = t * ct ;

    return pEe2H;
}

Vec3 QuadrupedLeg::calcPEe2H_B(Vec4 q){
    // 符号化处理后的机构参数
    float l1 = _sideSign * _abadLinkLength; // 侧摆机构有效长度（含左右符号）
    float l2 = _hipLinkLength;              // 大腿长度（方向补偿）
    float l3 = _kneeLinkLength;             // 小腿长度（方向补偿）

    // 关节三角函数预计算
    float s1 = std::sin(q(0));  // 外展角正弦
    float s2 = std::sin(q(1));  // 髋屈曲正弦
    float s3 = std::sin(q(2));  // 膝屈曲正弦
    float c1 = std::cos(q(0));  // 外展角余弦
    float c2 = std::cos(q(1));  // 髋屈曲余弦 
    float c3 = std::cos(q(2));  // 膝屈曲余弦

    // 关节角度组合项（q2+q3）
    float c23 = c2 * c1 - s2 * s1;  // cos(q2+q3)
    float s23 = s2 * c1 + c2 * s1;  // sin(q2+q3)

    float t = l3 * sin(M_PI * 30 / 180) + 15 * 180 * q(1) / M_PI; // 简化物理意义明确的表达式
    float st = sin(M_PI * 30 / 180); // 使用标准π值
    float ct = cos(M_PI * 30 / 180);

    Vec3 pEe2H;
    // X轴分量：前后方向运动合成
    pEe2H(0) = l1 * c1 + l2 * c23 + t * st * c23 ;  // 大腿与小腿的投影合成
    
    // Y轴分量：侧向运动合成（含外展机构影响）
    pEe2H(1) = l1 * s1 + l2 * s23 + t * st * s23 ;     // 小腿侧向分量

    // Z轴分量：垂直方向运动合成
    pEe2H(2) = t * ct;

    return pEe2H;
}

/**
 * 正运动学解算（机体坐标系）
 * @param q 关节角度向量 [q1, q2, q3] (rad)
 * @return 足端在机体坐标系的三维位置 [x, y, z]^T (m)
 * 
 * 实现原理：
 * 1. 调用calcPEe2H获取髋关节坐标系下的足端位置
 * 2. 叠加髋关节安装偏移量_pHip2B（从机体坐标系到髋关节坐标系的变换）
 */
Vec3 QuadrupedLeg::calcPEe2B(Vec3 q){
    return _pHip2B + calcPEe2H(q);
}

Vec3 QuadrupedLeg::calcPEe2B_F(Vec4 q){
    return _pHip2B + calcPEe2H_F(q);
}

Vec3 QuadrupedLeg::calcPEe2B_B(Vec4 q){
    return _pHip2B + calcPEe2H_B(q);
}

/**
 * 速度级正运动学计算（末端速度求解）
 * @param q 关节角度向量 [q1, q2, q3] (rad)
 * @param qd 关节速度向量 [qd1, qd2, qd3] (rad/s)
 * @return 足端末端速度向量 [vx, vy, vz] (m/s)
 * 
 * 核心公式：
 * v_ee = J(q) * qd
 * 其中J(q)为当前构型下的雅可比矩阵
 */
Vec3 QuadrupedLeg::calcVEe(Vec3 q, Vec3 qd){
    return calcJaco(q) * qd;
}

Vec3 QuadrupedLeg::calcVEe_F(Vec4 q, Vec4 qd){
    return calcJaco_F(q) * qd.head(3); 
}

Vec3 QuadrupedLeg::calcVEe_B(Vec4 q, Vec4 qd){ 
   return calcJaco_B(q) * qd.head(3); 
}


/**
 * 逆运动学解算核心算法
 * @param pEe 足端位置（HIP或BODY坐标系）
 * @param frame 坐标系类型枚举
 * @return 关节角度向量 [外展角, 屈曲角, 膝关节角]
 * 
 * 算法流程：
 * 1. 坐标系转换：将输入位置统一转换到髋关节坐标系
 * 2. 参数分解：提取足端坐标分量和机构参数
 * 3. 几何解算：分三步求解三个关节角
 * 
 * 安全机制：
 * - 坐标系类型校验
 * - 数值有效性检查（通过子函数实现）
 */
Vec3 QuadrupedLeg::calcQ(Vec3 pEe, FrameType frame){
    // 坐标系统一处理（核心第一步）
    Vec3 pEe2H;  // 足端在髋关节坐标系的位置
    if(frame == FrameType::HIP)
        pEe2H = pEe;  // 直接使用HIP坐标系输入
    else if(frame == FrameType::BODY)
        pEe2H = pEe - _pHip2B;  // 转换BODY坐标系到HIP坐标系
    else{
        std::cout << "[ERROR] The frame of QuadrupedLeg::calcQ can only be HIP or BODY!" << std::endl;
        exit(-1);
    }

    // 坐标分量分解（核心第二步）
    float px = pEe2H(0);  // 髋关节坐标系X分量（前后方向）
    float py = pEe2H(1);  // 髋关节坐标系Y分量（侧向）
    float pz = pEe2H(2);  // 髋关节坐标系Z分量（垂直方向）

    // 机构参数获取（基于机械设计参数）
    float b2y = _abadLinkLength * _sideSign;  // 髋关节侧向安装偏移
    float b3z = -_hipLinkLength;   // 大腿有效长度（方向补偿）
    float b4z = -_kneeLinkLength;   // 小腿有效长度（方向补偿）
    float a = _abadLinkLength;      // 髋关节到基座的距离

    // 几何关系建立（核心第三步）
    float c = sqrt(px*px + py*py + pz*pz);  // 空间直线距离
    float b = sqrt(c*c - a*a);      // 投影平面内的有效距离

    // 分步解析解算（核心第四步）
    float q1 = q1_ik(py, pz, b2y);  // 侧向平面关节角
    float q3 = q3_ik(b3z, b4z, b); // 矢状面膝关节角 
    float q2 = q2_ik(q1, q3, px, py, pz, b3z, b4z); // 三维空间髋关节角

    return Vec3(q1, q2, q3);  // 返回关节角向量
}

Vec4 QuadrupedLeg::calcQ_F(Vec3 pEe, FrameType frame){
    // 坐标系统一处理（核心第一步）
    Vec3 pEe2H;  // 足端在髋关节坐标系的位置
    if(frame == FrameType::HIP)
        pEe2H = pEe;  // 直接使用HIP坐标系输入
    else if(frame == FrameType::BODY)
        pEe2H = pEe - _pHip2B;  // 转换BODY坐标系到HIP坐标系
    else{
        std::cout << "[ERROR] The frame of QuadrupedLeg::calcQ can only be HIP or BODY!" << std::endl;
        exit(-1);
    }

    // 坐标分量分解（核心第二步）
    float px = pEe2H(0);  // 髋关节坐标系X分量（前后方向）
    float py = pEe2H(1);  // 髋关节坐标系Y分量（侧向）
    float pz = pEe2H(2);  // 髋关节坐标系Z分量（垂直方向）

    // 已知参数
    float l1 = _sideSign * _abadLinkLength;
    float l3 = _kneeLinkLength;
    float st = sin(M_PI/6); // sin(30°)
    float ct = cos(M_PI/6); // cos(30°)
    
    // Step 1: 计算q1
    float q1 = atan2(py, px);  // 从X/Y分量直接得到外展角
    
    // Step 2: 计算t
    float t = pz / ct;  // 从Z轴分量反推t
    
    // Step 3: 反推q2（原公式t = l3*st + 15*(180/π)*q2）
    float q2 = (t - l3*st) * M_PI / (15*180);  // 单位转换
    
    // q3在此模型中未参与计算，保持为0
    float q3 = 0;

    float q4 = 0;

    return Vec4(q1, q2, q3, q4);
}

Vec4 QuadrupedLeg::calcQ_B(Vec3 pEe, FrameType frame){
    // 坐标系统一处理（核心第一步）
    Vec3 pEe2H;  // 足端在髋关节坐标系的位置
    if(frame == FrameType::HIP)
        pEe2H = pEe;  // 直接使用HIP坐标系输入
    else if(frame == FrameType::BODY)
        pEe2H = pEe - _pHip2B;  // 转换BODY坐标系到HIP坐标系
    else{
        std::cout << "[ERROR] The frame of QuadrupedLeg::calcQ can only be HIP or BODY!" << std::endl;
        exit(-1);
    }

    // 坐标分量分解（核心第二步）
    float px = pEe2H(0);  // 髋关节坐标系X分量（前后方向）
    float py = pEe2H(1);  // 髋关节坐标系Y分量（侧向）
    float pz = pEe2H(2);  // 髋关节坐标系Z分量（垂直方向）

    // 机构参数
    float l1 = _sideSign * _abadLinkLength;
    float l2 = _hipLinkLength;
    float l3 = _kneeLinkLength;
    float st = sin(M_PI/6); // sin(30°)
    float ct = cos(M_PI/6); // cos(30°)

    // Step 1: 从Z轴解算t和q2
    float t = pz / ct;
    float q2 = (t - l3*st) * M_PI / (15*180); // 单位转换

    // Step 2: 计算合成半径R
    float R = l2 + t*st;

    // Step 3: 解算外展角q1
    float q1 = atan2(py - l1*sin(q1), px - l1*cos(q1)); // 需要迭代解算
    
    // Step 4: 解算q23 = q2 + q3
    float X_proj = px - l1*cos(q1);
    float Y_proj = py - l1*sin(q1);
    float q23 = atan2(Y_proj, X_proj);

    // Step 5: 计算q3
    float q3 = q23 - q2;

    // Step 6: 计算q4
    float q4 = 0;

    return Vec4(q1, q2, q3 ,q4);
}

/**
 * 逆速度运动学解算（已知关节角度）
 * @param q 当前关节角度向量 [q1, q2, q3] (rad)
 * @param vEe 足端末端速度向量 [vx, vy, vz] (m/s)
 * @return 期望关节速度向量 [qd1, qd2, qd3] (rad/s)
 * 
 * 原理公式：qd = J(q)^{-1} * vEe
 * 注意：当雅可比矩阵接近奇异时，需要做正则化处理
 */
Vec3 QuadrupedLeg::calcQd(Vec3 q, Vec3 vEe){
    return calcJaco(q).inverse() * vEe;
}

Vec4 QuadrupedLeg::calcQd_F(Vec4 q, Vec3 vEe){
    
    Vec4 qd_full = Vec4::Zero();
    qd_full.head(3) = calcJaco_F(q).inverse() * vEe;  // 仅计算前3个关节
    
    return qd_full;
}

Vec4 QuadrupedLeg::calcQd_B(Vec4 q, Vec3 vEe){

    Vec4 qd_full = Vec4::Zero();
    qd_full.head(3) = calcJaco_B(q).inverse() * vEe;
    
    return qd_full;
}

/**
 * 逆速度运动学解算（已知足端位置）
 * @param pEe 足端位置向量 [x, y, z] (m)
 * @param vEe 足端末端速度向量 [vx, vy, vz] (m/s)
 * @param frame 坐标系类型（HIP或BODY）
 * @return 期望关节速度向量 [qd1, qd2, qd3] (rad/s)
 * 
 * 实现流程：
 * 1. 通过逆运动学计算当前关节角度
 * 2. 调用基于角度的逆速度解算
 */
Vec3 QuadrupedLeg::calcQd(Vec3 pEe, Vec3 vEe, FrameType frame){
    Vec3 q = calcQ(pEe, frame);
    return calcJaco(q).inverse() * vEe;
}

Vec4 QuadrupedLeg::calcQd_F(Vec3 pEe, Vec3 vEe, FrameType frame){
    Vec4 q = calcQ_F(pEe, frame);  
    Vec4 qd_full = Vec4::Zero();    
    qd_full.head(3) = calcJaco_F(q).inverse() * vEe; // 自动补零扩展维度
    return qd_full;
}

Vec4 QuadrupedLeg::calcQd_B(Vec3 pEe, Vec3 vEe, FrameType frame){
    Vec4 q = calcQ_B(pEe, frame);  
    Vec4 qd_full = Vec4::Zero();    
    qd_full.head(3) = calcJaco_B(q).inverse() * vEe; // 自动补零扩展维度
    return  qd_full;
}


/**
 * 静力学映射（末端力到关节力矩）
 * @param q 当前关节角度向量 [q1, q2, q3] (rad)
 * @param force 足端作用力向量 [fx, fy, fz] (N)
 * @return 等效关节力矩向量 [tau1, tau2, tau3] (Nm)
 * 
 * 原理公式：τ = J(q)^T * F
 * 应用场景：阻抗控制、力控应用
 */
Vec3 QuadrupedLeg::calcTau(Vec3 q, Vec3 force){
    return calcJaco(q).transpose() * force;
}

Vec4 QuadrupedLeg::calcTau_F(Vec4 q, Vec3 force){
    Vec4 tau_full = Vec4::Zero();
    tau_full.head(3) = calcJaco_F(q).transpose() * force;
    return tau_full;
}

Vec4 QuadrupedLeg::calcTau_B(Vec4 q, Vec3 force){
    Vec4 tau_full = Vec4::Zero();
    tau_full.head(3) = calcJaco_B(q).transpose() * force;
    return tau_full;
}

/**
 * 计算雅可比矩阵（关节空间到操作空间的微分映射）
 * @param q 关节角度向量 [q1, q2, q3] (rad)
 * @return 3x3雅可比矩阵，满足 v = J * qd
 * 
 * 矩阵物理意义：
 * [dx/dq1  dx/dq2  dx/dq3]
 * [dy/dq1  dy/dq2  dy/dq3]
 * [dz/dq1  dz/dq2  dz/dq3]
 * 
 * 实现方法：
 * 通过解析法推导的雅可比矩阵闭合解
 */
Mat3 QuadrupedLeg::calcJaco(Vec3 q){
    Mat3 jaco;

    // 机构参数初始化（带符号处理）
    float l1 = _abadLinkLength * _sideSign;  // 髋关节侧向有效长度
    float l2 = -_hipLinkLength;              // 大腿长度（方向补偿）
    float l3 = -_kneeLinkLength;              // 小腿长度（方向补偿）

    // 关节三角函数预计算
    float s1 = std::sin(q[0]);  // 髋外展角正弦
    float s2 = std::sin(q[1]);  // 髋屈曲角正弦
    float s3 = std::sin(q[2]);  // 膝屈曲角正弦

    float c1 = std::cos(q[0]);  // 髋外展角余弦
    float c2 = std::cos(q[1]);  // 髋屈曲角余弦
    float c3 = std::cos(q[2]);  // 膝屈曲角余弦

    // 关节角度组合项
    float c23 = c2 * c3 - s2 * s3;  // cos(q2+q3)
    float s23 = s2 * c3 + c2 * s3;  // sin(q2+q3)


    // 雅可比矩阵元素填充（按列优先顺序）
    // 第一列：对q1（髋外展）的偏导
    jaco(0, 0) = 0;  // x方向无影响
    jaco(1, 0) = -l3*c1*c23 - l2*c1*c2 - l1*s1;  // y方向影响项
    jaco(2, 0) = -l3*s1*c23 - l2*c2*s1 + l1*c1;  // z方向影响项

    // 第二列：对q2（髋屈曲）的偏导
    jaco(0, 1) = l3*c23 + l2*c2;        // x方向影响项
    jaco(1, 1) = l3*s1*s23 + l2*s1*s2;  // y方向影响项
    jaco(2, 1) = -l3*c1*s23 - l2*c1*s2; // z方向影响项

    // 第三列：对q3（膝屈曲）的偏导 
    jaco(0, 2) = l3*c23;        // x方向影响项
    jaco(1, 2) = l3*s1*s23;     // y方向影响项
    jaco(2, 2) = -l3*c1*s23;    // z方向影响项

    return jaco;
}

Mat3 QuadrupedLeg::calcJaco_F(Vec4 q){
    Mat3 jaco;
    
    // 机构参数
    float l1 = _sideSign * _abadLinkLength;
    float l3 = _kneeLinkLength;
    float st = sin(M_PI/6); // sin(30°)
    float ct = cos(M_PI/6); // cos(30°)
    
    // 关节三角函数
    float s1 = sin(q(0));
    float c1 = cos(q(0));
    
    // 中间参数
    float t = l3*st + 15*(180/M_PI)*q(1);
    float dt_dq2 = 15*(180/M_PI); // t对q2的导数

    // 雅可比矩阵元素填充
    // 第一列：对q1的偏导
    jaco(0,0) = -l1*s1 - t*st*s1; // ∂X/∂q1
    jaco(1,0) = l1*c1 + t*st*c1;  // ∂Y/∂q1
    jaco(2,0) = 0;                // ∂Z/∂q1

    // 第二列：对q2的偏导
    jaco(0,1) = dt_dq2 * st * c1; // ∂X/∂q2
    jaco(1,1) = dt_dq2 * st * s1; // ∂Y/∂q2
    jaco(2,1) = dt_dq2 * ct;      // ∂Z/∂q2

    // 第三列：对q3的偏导（该模型未使用q3）
    jaco(0,2) = 0;
    jaco(1,2) = 0;
    jaco(2,2) = 0;

    return jaco;
}

Mat3 QuadrupedLeg::calcJaco_B(Vec4 q){
    Mat3 jaco;
    
    // 机构参数
    float l1 = _sideSign * _abadLinkLength;
    float l2 = _hipLinkLength;
    float l3 = _kneeLinkLength;
    float st = sin(M_PI/6); // sin(30°)
    float ct = cos(M_PI/6); // cos(30°)
    
    // 关节角度
    float q1 = q(0);
    float q2 = q(1);
    float q3 = q(2);
    
    // 三角函数预计算
    float s1 = sin(q1), c1 = cos(q1);
    float s23 = sin(q2+q3), c23 = cos(q2+q3);
    
    // 中间参数
    float t = l3*st + 15*(180/M_PI)*q2;
    float dt_dq2 = 15*(180/M_PI); // t对q2的导数

    // 雅可比矩阵元素计算
    // 第一列：对q1的偏导
    jaco(0,0) = -l1*s1 + (-l2*s23 - t*st*s23)*(-s1); // ∂X/∂q1
    jaco(1,0) = l1*c1 + (l2*c23 + t*st*c23)*(-s1);   // ∂Y/∂q1
    jaco(2,0) = 0;                                   // ∂Z/∂q1

    // 第二列：对q2的偏导
    jaco(0,1) = (-l2*s23 - t*st*s23) + dt_dq2*st*c23; // ∂X/∂q2
    jaco(1,1) = (l2*c23 + t*st*c23) + dt_dq2*st*s23;  // ∂Y/∂q2
    jaco(2,1) = dt_dq2 * ct;                          // ∂Z/∂q2

    // 第三列：对q3的偏导
    jaco(0,2) = -l2*s23 - t*st*s23; // ∂X/∂q3
    jaco(1,2) = l2*c23 + t*st*c23;  // ∂Y/∂q3
    jaco(2,2) = 0;                  // ∂Z/∂q3


    return jaco;
}

/**
 * 髋关节外展角解析计算（侧向平面投影法）
 * @param py 足端在髋坐标系Y坐标 (m)
 * @param pz 足端在髋坐标系Z坐标 (m)
 * @param l1 髋关节侧向偏移量 (m)
 * @return 髋关节外展角 q1 (rad)
 * 
 * 计算原理：
 * 1. 将三维问题投影到侧向平面（Y-Z平面）
 * 2. 构建几何三角形关系
 * 3. 通过三角函数解析求解关节角度
 * 
 * 几何关系：
 *       /|
 *    L / | pz
 *     /  |
 *    /___|
 *    l1  py
 */
float QuadrupedLeg::q1_ik(float py, float pz, float l1){
    float q1;
    float L = sqrt(pow(py,2)+pow(pz,2)-pow(l1,2));
    q1 = atan2(pz*l1+py*L, py*l1-pz*L);
    return q1;
}


/**
 * 膝关节角度解析计算（三角形解法）
 * @param b3z 大腿连杆长度（Z方向）(m)
 * @param b4z 小腿连杆长度（Z方向）(m)
 * @param b 足端在运动平面内的投影距离 (m)
 * @return 膝关节屈曲角度 q3 (rad)
 * 
 * 计算原理：
 * 1. 基于余弦定理建立连杆三角形关系
 * 2. 处理计算中的数值稳定性问题
 * 3. 角度方向适配实际机械结构
 * 
 * 几何关系：
 *        b4z
 *     +--------[足端]
 *     | \
 *     |  \ 
 *  b  |   \ 
 *     |    \
 *     +-----+
 *       b3z
 */
float QuadrupedLeg::q3_ik(float b3z, float b4z, float b){
    float q3, temp;

    // 余弦定理计算关节角余弦值
    temp = (pow(b3z, 2) + pow(b4z, 2) - pow(b, 2))/(2*fabs(b3z*b4z));

    // 数值稳定性处理（防止反余弦超出定义域）   
    if(temp>1) temp = 1;
    if(temp<-1) temp = -1;

    q3 = acos(temp);            // 基础角度计算
    q3 = -(M_PI - q3);          // 转换为膝关节实际运动方向
    return q3;                  // 返回角度范围：0 ~ π (实际物理限制)
}


/**
 * 髋关节屈曲角解析计算（平面几何法）
 * @param q1 已计算的髋外展角 (rad)
 * @param q3 已计算的膝屈曲角 (rad)
 * @param px 足端在髋坐标系X坐标 (m)
 * @param py 足端在髋坐标系Y坐标 (m)
 * @param pz 足端在髋坐标系Z坐标 (m)
 * @param b3z 大腿连杆Z向长度 (m)
 * @param b4z 小腿连杆Z向长度 (m)
 * @return 髋关节屈曲角 q2 (rad)
 * 
 * 计算原理：
 * 1. 将三维问题投影到矢状平面（Sagittal Plane）
 * 2. 通过平面三角关系建立方程组
 * 3. 使用atan2函数保证角度象限正确性
 * 
 * 几何关系：
 * a1 = py*sin(q1) - pz*cos(q1)  # 足端在侧向平面的投影
 * a2 = px                       # 足端在前进方向的投影
 * m1 = L2*sin(q3)               # 膝关节投影分量
 * m2 = L1 + L2*cos(q3)          # 大腿与小腿的合成长度
 */
float QuadrupedLeg::q2_ik(float q1, float q3, float px, float py, float pz, float b3z, float b4z){
    float q2, a1, a2, m1, m2;
    
    // 平面投影计算
    a1 = py*sin(q1) - pz*cos(q1);   // 侧向平面投影（消除髋关节外展影响）
    a2 = px;                        // 前进方向保持原始坐标

    // 连杆几何参数合成   
    m1 = b4z*sin(q3);               // 膝关节屈曲产生的垂直分量
    m2 = b3z + b4z*cos(q3);         // 大腿与小腿的水平合成长度

    // 平面三角解析解    
    q2 = atan2(m1*a1+m2*a2,         // 正弦项分子
                 m1*a2-m2*a1);      // 余弦项分子
    return q2;
}

/**
 * 计算浮心位置（相对于足端）
 * @param pEe 足端位置
 * @return 浮心位置（与足端同一坐标系）
 * 
 * 实现原理：
 * 浮心位置 = 足端位置 + 浮心安装偏移量
 * 偏移量方向由机械参数_BuoyancyQ决定
 */
Vec3 QuadrupedLeg::calcBuoyancyCenter(Vec3 pEe) {
    // 考虑腿部左右侧符号的偏移量
    Vec3 offset = _BuoyancyQ;
    offset(1) *= _sideSign; // Y方向受侧摆影响
    return pEe + offset * _BuoyancyCenterLength;
}

/**
 * 计算质心位置（相对于足端）
 * @param pEe 足端位置
 * @return 质心位置（与足端同一坐标系）
 * 
 * 实现原理：
 * 质心位置 = 足端位置 + 质心安装偏移量
 * 偏移量方向由机械参数_MassQ决定
 */
Vec3 QuadrupedLeg::calcMassCenter(Vec3 pEe) {
    // 考虑腿部左右侧符号的偏移量
    Vec3 offset = _MassQ;
    offset(1) *= _sideSign; // Y方向受侧摆影响
    return pEe + offset * _MassCenterLength;
}

// 在现有类中添加坐标系转换方法
Vec3 QuadrupedLeg::calcPEe2H_Mass(Vec3 q) {
    Vec3 pEe = calcPEe2H(q);
    return calcMassCenter(pEe);
}

Vec3 QuadrupedLeg::calcPEe2H_Buoyancy(Vec3 q) {
    Vec3 pEe = calcPEe2H(q);
    return calcBuoyancyCenter(pEe);
}

// 新增质心速度计算
Vec3 QuadrupedLeg::calcVEe_Mass(Vec3 q, Vec3 qd){
    Mat3 J_m = calcJaco(q); // 获取足端雅可比
    Vec3 offset = _MassQ * _MassCenterLength; // 质心偏移量
    offset(1) *= _sideSign;
    
    // 速度转换公式：v_m = v_ee + ω × offset
    // 简化为：v_m ≈ J_m * qd + J_rot × offset
    return J_m * qd + skew(offset) * (J_m.block(0,0,3,3).inverse() * qd);
}

// 新增浮心速度计算
Vec3 QuadrupedLeg::calcVEe_Buoyancy(Vec3 q, Vec3 qd){
    Mat3 J_b = calcJaco(q); // 足端雅可比
    Vec3 offset = _BuoyancyQ * _BuoyancyCenterLength;
    offset(1) *= _sideSign;
    
    return J_b * qd + skew(offset) * (J_b.block(0,0,3,3).inverse() * qd);
}

// 质心力矩映射
Vec3 QuadrupedLeg::calcTau_Mass(Vec3 q, Vec3 force){
    Mat3 J_m = calcJaco(q);
    Vec3 offset = _MassQ * _MassCenterLength;
    offset(1) *= _sideSign;
    
    // 力矩转换公式：τ = J_m^T * (F + offset × F_rot)
    return J_m.transpose() * (force + skew(offset) * force);
}

// 浮心力矩映射
Vec3 QuadrupedLeg::calcTau_Buoyancy(Vec3 q, Vec3 force){
    Mat3 J_b = calcJaco(q);
    Vec3 offset = _BuoyancyQ * _BuoyancyCenterLength;
    offset(1) *= _sideSign;
    
    return J_b.transpose() * (force + skew(offset) * force);
}