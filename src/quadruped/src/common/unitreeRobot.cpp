
#include "common/unitreeRobot.h"
#include <iostream>

/**
 * 获取参考腿（默认右前腿）在机体坐标系下的足端位置
 * @param state 包含关节角度信息的底层状态
 * @return 三维位置向量 [x, y, z]^T (m)
 * 
 * 功能说明：
 * - 作为其他计算（如支撑多边形）的基准参考点
 * - 固定选择0号腿（右前腿）作为参考腿
 */
Vec3 QuadrupedRobot::getX_XM(LowlevelState_XM &state){
    return getFootPosition_XM(state, 0, FrameType::BODY);
}

Vec3 QuadrupedRobot::getX(LowlevelState &state){
    return getFootPosition(state, 0, FrameType::BODY);
}

/**
 * 计算所有足端相对于参考腿的位置矩阵
 * @param state 底层状态数据（包含关节角度）
 * @return 3x4位置矩阵，每列对应一个足端相对于参考腿的位移
 * 
 * 应用场景：
 * - 支撑多边形计算
 * - 平衡稳定性判断
 */
Vec34 QuadrupedRobot::getVecXP_XM(LowlevelState_XM &state){
    Vec3 x = getX_XM(state);
    Vec34 vecXP;
    Mat4  qLegs;
    qLegs = state.getQ();

    for(int i(0); i < 4; ++i){
        // 根据腿ID选择计算方法：0,2号腿用F模型，1,3号腿用B模型
        if(i == 0 || i == 2){
            vecXP.col(i) = _Legs[i]->calcPEe2B_F(qLegs.col(i)) - x;
        }else{
            vecXP.col(i) = _Legs[i]->calcPEe2B_B(qLegs.col(i)) - x;
        }
    }
    return vecXP;
}

Vec34 QuadrupedRobot::getVecXP(LowlevelState &state){
    Vec3 x = getX(state);
    Vec34 vecXP, qLegs;
    qLegs = state.getQ();

    for(int i(0); i < 4; ++i){
        vecXP.col(i) = _Legs[i]->calcPEe2B(qLegs.col(i)) - x;
    }
    return vecXP;
}



/**
 * 逆运动学解算：根据足端目标位置计算关节角度
 * @param vecP 足端目标位置矩阵（4列分别对应四条腿）
 * @param frame 坐标系类型（BODY/HIP）
 * @return 12维关节角度向量 [腿0髋, 腿0膝, 腿0踝, 腿1髋...] 
 */
Vec16 QuadrupedRobot::getQ_XM(const Vec34 &vecP, FrameType frame){
    Vec16 q;
    for(int i(0); i < 2; ++i){
        // 每条腿单独计算逆运动学（4个关节）
        q.segment(4*i, 4) = _Legs[i]->calcQ_F(vecP.col(i), frame); 
    }

    for(int i(2); i < 4; ++i){
        // 每条腿单独计算逆运动学（4个关节）
        q.segment(4*i, 4) = _Legs[i]->calcQ_B(vecP.col(i), frame);
    }
    return q;
}

Vec12 QuadrupedRobot::getQ(const Vec34 &vecP, FrameType frame){
    Vec12 q;
    for(int i(0); i < 4; ++i){
        q.segment(3*i, 3) = _Legs[i]->calcQ(vecP.col(i), frame);
    }
    return q;
}

/**
 * 逆运动学速度解算：根据足端目标位置和速度计算关节角速度
 * @param pos 足端目标位置矩阵（4列对应四条腿）
 * @param vel 足端目标速度矩阵
 * @param frame 坐标系类型（BODY/HIP）
 * @return 12维关节角速度向量 [rad/s]
 * 
 * 实现原理：
 * 对每条腿分别进行逆运动学速度解算，组合成12维向量
 */
Vec16 QuadrupedRobot::getQd_XM(const Vec34 &pos, const Vec34 &vel, FrameType frame){
    Vec16 qd;
    for(int i(0); i < 2; ++i){
        qd.segment(4*i, 4) = _Legs[i]->calcQd_F(pos.col(i), vel.col(i), frame);
    }

    for(int i(2); i < 4; ++i){
        qd.segment(4*i, 4) = _Legs[i]->calcQd_B(pos.col(i), vel.col(i), frame);
    }
    return qd;
}

Vec12 QuadrupedRobot::getQd(const Vec34 &pos, const Vec34 &vel, FrameType frame){
    Vec12 qd;
    for(int i(0); i < 4; ++i){
        qd.segment(3*i, 3) = _Legs[i]->calcQd(pos.col(i), vel.col(i), frame);
    }
    return qd;
}

/**
 * 计算关节力矩（基于逆动力学）
 * @param q 当前关节角度（12维向量）
 * @param feetForce 4x3足端接触力矩阵（每列为对应腿的三维力向量）
 * @return 12维关节力矩向量 [Nm]
 * 
 * 实现原理：
 * 使用雅可比矩阵转置法：τ = J^T * F
 * 其中J为运动学雅可比矩阵，F为足端接触力
 */
Vec16 QuadrupedRobot::getTau_XM(const Vec16 &q, const Vec34 feetForce){
    Vec16 tau;
    for(int i(0); i < 2; ++i){
        tau.segment(4*i, 4) = _Legs[i]->calcTau_F(q.segment(4*i, 4), feetForce.col(i));
    }

    for(int i(2); i < 4; ++i){
        tau.segment(4*i, 4) = _Legs[i]->calcTau_B(q.segment(4*i, 4), feetForce.col(i));
    }

    return tau;
}

Vec12 QuadrupedRobot::getTau(const Vec12 &q, const Vec34 feetForce){
    Vec12 tau;
    for(int i(0); i < 4; ++i){
        tau.segment(3*i, 3) = _Legs[i]->calcTau(q.segment(3*i, 3), feetForce.col(i));
    }
    return tau;
}

/**
 * 获取指定腿足端在指定坐标系中的位置
 * @param state 底层状态数据（包含关节角度信息）
 * @param id 腿的索引（0-3对应四条腿）
 * @param frame 坐标系类型：BODY(机体坐标系)/HIP(髋关节坐标系)
 * @return 三维位置向量 [x, y, z]^T (m)
 * 
 * 实现原理：
 * 1. 通过state获取当前所有关节角度
 * 2. 根据坐标系类型调用不同运动学计算：
 *    - BODY：计算机体坐标系下的足端位置
 *    - HIP：计算髋关节坐标系下的足端位置
 * 3. 错误坐标系类型时触发程序终止
 */
Vec3 QuadrupedRobot::getFootPosition_XM(LowlevelState_XM &state, int id, FrameType frame){
    Mat4 qLegs= state.getQ();// 获取当前所有关节角度
    Vec3 pEe;

    // 根据腿ID选择计算模型
    if(id == 0 || id == 2){
        pEe = (frame == FrameType::BODY) ? 
              _Legs[id]->calcPEe2B_F(qLegs.col(id)) :  // 右前腿和右后腿使用F模型
              _Legs[id]->calcPEe2H_F(qLegs.col(id));
    }else if(id == 1 || id == 3){
        pEe = (frame == FrameType::BODY) ? 
              _Legs[id]->calcPEe2B_B(qLegs.col(id)) :  // 左前腿和左后腿使用B模型
              _Legs[id]->calcPEe2H_B(qLegs.col(id));
    }else{
        std::cout << "[ERROR] Invalid leg ID: " << id << std::endl;
        exit(-1);
    }

    return pEe; // 返回最终计算结果
}

Vec3 QuadrupedRobot::getFootPosition(LowlevelState &state, int id, FrameType frame){
    Vec34 qLegs= state.getQ();

    if(frame == FrameType::BODY){
        return _Legs[id]->calcPEe2B(qLegs.col(id));
    }else if(frame == FrameType::HIP){
        return _Legs[id]->calcPEe2H(qLegs.col(id));
    }else{
        std::cout << "[ERROR] The frame of function: getFootPosition can only be BODY or HIP." << std::endl;
        exit(-1);
    }
}

Vec3 QuadrupedRobot::getBuoyancyCenterPosition(LowlevelState_XM &state, int id, FrameType frame){
    Mat4 qLegs= state.getQ();
    Vec3 pEe; // 足端基准位置

    // 根据腿ID选择计算模型
    if(id == 0 || id == 2){
        pEe = (frame == FrameType::BODY) ? 
              _Legs[id]->calcPEe2B_F(qLegs.col(id)) : 
              _Legs[id]->calcPEe2H_F(qLegs.col(id));
    }else if(id == 1 || id == 3){
        pEe = (frame == FrameType::BODY) ? 
              _Legs[id]->calcPEe2B_B(qLegs.col(id)) : 
              _Legs[id]->calcPEe2H_B(qLegs.col(id));
    }

    // 计算浮心位置（足端位置+偏移量）
    return _Legs[id]->calcBuoyancyCenter(pEe);
}

Vec3 QuadrupedRobot::getMassCenterPosition(LowlevelState_XM &state, int id, FrameType frame){
    Mat4 qLegs= state.getQ();
    Vec3 pEe;

    // 分支选择计算模型
    if(id == 0 || id == 2){
        pEe = (frame == FrameType::BODY) ? 
              _Legs[id]->calcPEe2B_F(qLegs.col(id)) : 
              _Legs[id]->calcPEe2H_F(qLegs.col(id));
    }else{
        pEe = (frame == FrameType::BODY) ? 
              _Legs[id]->calcPEe2B_B(qLegs.col(id)) : 
              _Legs[id]->calcPEe2H_B(qLegs.col(id));
    }

    // 转换到质心位置
    return _Legs[id]->calcMassCenter(pEe);
}

/**
 * 获取指定腿足端在机体坐标系下的速度
 * @param state 包含关节角度和关节角速度信息的底层状态
 * @param id 腿的索引（0 - 3 对应四条腿）
 * @return 三维速度向量 [vx, vy, vz]^T (m/s)，表示足端在机体坐标系下的速度
 * 
 * 实现原理：
 * 1. 从 state 中获取所有关节的角度和角速度
 * 2. 调用对应腿对象的 calcVEe 方法，计算该腿足端在机体坐标系下的速度
 */
Vec3 QuadrupedRobot::getFootVelocity_XM(LowlevelState_XM &state, int id){
    // 从底层状态中获取当前所有关节的角度，存储在 qLegs 中
    Mat4 qLegs = state.getQ();         // 获取当前所有关节角度
    // 从底层状态中获取当前所有关节的角速度，存储在 qdLegs 中
    Mat4 qdLegs= state.getQd();        // 获取当前所有关节角速度
    // 根据腿ID选择计算模型
    if(id == 0 || id == 2){
        return _Legs[id]->calcVEe_F(qLegs.col(id), qdLegs.col(id));
    }else if(id == 1 || id == 3){
        return _Legs[id]->calcVEe_B(qLegs.col(id), qdLegs.col(id));
    }else{
        std::cout << "[ERROR] Invalid leg ID: " << id << std::endl;
        exit(-1);
    }
}

Vec3 QuadrupedRobot::getBuoyancyCenterVelocity(LowlevelState_XM &state, int id){
    // 从底层状态中获取当前所有关节的角度，存储在 qLegs 中
    Mat4 qLegs = state.getQ();         // 获取当前所有关节角度
    // 从底层状态中获取当前所有关节的角速度，存储在 qdLegs 中
    Mat4 qdLegs= state.getQd();        // 获取当前所有关节角速度
    // 调用对应腿对象的 calcVEe 方法，传入该腿的关节角度和角速度，计算足端在机体坐标系下的速度并返回
    Vec3 vEe; // 足端基准速度
    Vec3 offset;

    // 根据腿ID选择计算模型
    if(id == 0 || id == 2){
        vEe = _Legs[id]->calcVEe_F(qLegs.col(id), qdLegs.col(id)); // 前腿使用F模型
    }else{
        vEe = _Legs[id]->calcVEe_B(qLegs.col(id), qdLegs.col(id)); // 后腿使用B模型
    }

    if(id == 0 || id == 2){
        offset = _Legs[id]->calcBuoyancyCenter(Vec3::Zero()) - _Legs[id]->calcPEe2B_F(qLegs.col(id));
    }else{
        offset = _Legs[id]->calcBuoyancyCenter(Vec3::Zero()) - _Legs[id]->calcPEe2B_B(qLegs.col(id));
    }

    return vEe + skew(state.getGyro()) * offset;
}

Vec3 QuadrupedRobot::getMassCenterVelocity(LowlevelState_XM &state, int id){
    // 从底层状态中获取当前所有关节的角度，存储在 qLegs 中
    Mat4 qLegs = state.getQ();         // 获取当前所有关节角度
    // 从底层状态中获取当前所有关节的角速度，存储在 qdLegs 中
    Mat4 qdLegs= state.getQd();        // 获取当前所有关节角速度
    // 调用对应腿对象的 calcVEe 方法，传入该腿的关节角度和角速度，计算足端在机体坐标系下的速度并返回
    Vec3 vEe; // 足端基准速度
    Vec3 offset;

    // 根据腿ID选择计算模型
    if(id == 0 || id == 2){
        vEe = _Legs[id]->calcVEe_F(qLegs.col(id), qdLegs.col(id)); // 前腿使用F模型
    }else{
        vEe = _Legs[id]->calcVEe_B(qLegs.col(id), qdLegs.col(id)); // 后腿使用B模型
    }

    if(id == 0 || id == 2){
        offset = _Legs[id]->calcMassCenter(Vec3::Zero()) - _Legs[id]->calcPEe2B_F(qLegs.col(id));
    }else{
        offset = _Legs[id]->calcMassCenter(Vec3::Zero()) - _Legs[id]->calcPEe2B_B(qLegs.col(id));
    }

    return vEe + skew(state.getGyro()) * offset;
}

Vec3 QuadrupedRobot::getFootVelocity(LowlevelState &state, int id){
    Vec34 qLegs = state.getQ();
    Vec34 qdLegs= state.getQd();
    return _Legs[id]->calcVEe(qLegs.col(id), qdLegs.col(id));
}

Vec34 QuadrupedRobot::getFeet2BPositions_XM(LowlevelState_XM &state, FrameType frame){
    // 定义一个3x4的矩阵，用于存储四条腿足端在指定坐标系下的位置
    Vec34 feetPos; 
    
    if(frame == FrameType::GLOBAL){
        for(int i(0); i<4; ++i){
            // 先获取每条腿足端在机体坐标系下的位置
            feetPos.col(i) = getFootPosition_XM(state, i, FrameType::BODY);
        }
        // 将机体坐标系下的位置通过旋转矩阵转换到全局坐标系
        feetPos = state.getRotMat() * feetPos;
    }

    // 若目标坐标系为机体坐标系（BODY）或髋关节坐标系（HIP）
    else if((frame == FrameType::BODY) || (frame == FrameType::HIP)){
        // 遍历四条腿
        for(int i(0); i<4; ++i){
            // 直接获取每条腿足端在指定坐标系下的位置
            feetPos.col(i) = getFootPosition_XM(state, i, frame);
        }
    }
    else{
        std::cout << "[ERROR] Frame error of function getFeet2BPositions" << std::endl;
        exit(-1);
    }
    return feetPos;
}


Vec34 QuadrupedRobot::getBuoyancyCenter2BPositions(LowlevelState_XM &state, FrameType frame){
    // 定义一个3x4的矩阵，用于存储四条腿足端在指定坐标系下的位置
    Vec34 feetPos; 
    
    if(frame == FrameType::GLOBAL){
        for(int i(0); i<4; ++i){
            // 先获取每条腿足端在机体坐标系下的位置
            feetPos.col(i) = getBuoyancyCenterPosition(state, i, FrameType::BODY);
        }
        // 将机体坐标系下的位置通过旋转矩阵转换到全局坐标系
        feetPos = state.getRotMat() * feetPos;
    }

    // 若目标坐标系为机体坐标系（BODY）或髋关节坐标系（HIP）
    else if((frame == FrameType::BODY) || (frame == FrameType::HIP)){
        // 遍历四条腿
        for(int i(0); i<4; ++i){
            // 直接获取每条腿足端在指定坐标系下的位置
            feetPos.col(i) = getBuoyancyCenterPosition(state, i, frame);
        }
    }
    else{
        std::cout << "[ERROR] Frame error of function getFeet2BPositions" << std::endl;
        exit(-1);
    }
    return feetPos;
}

Vec34 QuadrupedRobot::getMassCenter2BPositions(LowlevelState_XM &state, FrameType frame){
        // 定义一个3x4的矩阵，用于存储四条腿足端在指定坐标系下的位置
    Vec34 feetPos; 
    
    if(frame == FrameType::GLOBAL){
        for(int i(0); i<4; ++i){
            // 先获取每条腿足端在机体坐标系下的位置
            feetPos.col(i) = getBuoyancyCenterPosition(state, i, FrameType::BODY);
        }
        // 将机体坐标系下的位置通过旋转矩阵转换到全局坐标系
        feetPos = state.getRotMat() * feetPos;
    }

    // 若目标坐标系为机体坐标系（BODY）或髋关节坐标系（HIP）
    else if((frame == FrameType::BODY) || (frame == FrameType::HIP)){
        // 遍历四条腿
        for(int i(0); i<4; ++i){
            // 直接获取每条腿足端在指定坐标系下的位置
            feetPos.col(i) = getBuoyancyCenterPosition(state, i, frame);
        }
    }
    else{
        std::cout << "[ERROR] Frame error of function getFeet2BPositions" << std::endl;
        exit(-1);
    }
    return feetPos;
}


Vec34 QuadrupedRobot::getFeet2BPositions(LowlevelState &state, FrameType frame){
    Vec34 feetPos;
    if(frame == FrameType::GLOBAL){
        for(int i(0); i<4; ++i){
            feetPos.col(i) = getFootPosition(state, i, FrameType::BODY);
        }
        feetPos = state.getRotMat() * feetPos;
    }
    else if((frame == FrameType::BODY) || (frame == FrameType::HIP)){
        for(int i(0); i<4; ++i){
            feetPos.col(i) = getFootPosition(state, i, frame);
        }
    }
    else{
        std::cout << "[ERROR] Frame error of function getFeet2BPositions" << std::endl;
        exit(-1);
    }
    return feetPos;
}

/**
 * 获取所有足端在指定坐标系中的速度
 * @param state 包含关节角度和角速度的底层状态
 * @param frame 目标坐标系类型（GLOBAL/BODY/HIP）
 * @return 3x4速度矩阵，每列对应一个足端的三维速度向量
 * 
 * 实现原理：
 * 1. 基础速度计算：通过正向运动学导数获取各足端在机体坐标系中的速度
 * 2. 坐标系处理：
 *    - GLOBAL：考虑机体旋转带来的速度分量，转换到世界坐标系
 *    - BODY/HIP：直接返回机体/髋关节坐标系下的速度
 * 
 * 关键公式：
 * 全局速度 = 旋转矩阵 × (基础速度 + 角速度叉乘位置)
 */
Vec34 QuadrupedRobot::getFeet2BVelocities_XM(LowlevelState_XM &state, FrameType frame){
    // 定义一个3x4的矩阵，用于存储四条腿足端在指定坐标系下的速度
    Vec34 feetVel;
    // 获取各足端在机体坐标系中的基础速度
    for(int i(0); i<4; ++i){
        feetVel.col(i) = getFootVelocity_XM(state, i);
    }

    if(frame == FrameType::GLOBAL){
        // 计算旋转带来的附加速度分量
        Vec34 feetPos = getFeet2BPositions_XM(state, FrameType::BODY);
        feetVel += skew(state.getGyro()) * feetPos;// 角速度叉乘位置项

        // 转换到全局坐标系
        return state.getRotMat() * feetVel;
    }
    else if((frame == FrameType::BODY) || (frame == FrameType::HIP)){
        return feetVel;// 直接返回机体/髋关节坐标系速度
    }
    else{
        std::cout << "[ERROR] Frame error of function getFeet2BVelocities" << std::endl;
        exit(-1);
    }   
}

Vec34 QuadrupedRobot::getBuoyancyCenter2BVelocities(LowlevelState_XM &state, FrameType frame){
    // 定义一个3x4的矩阵，用于存储四条腿足端在指定坐标系下的速度
    Vec34 feetVel;
    // 获取各足端在机体坐标系中的基础速度
    for(int i(0); i<4; ++i){
        feetVel.col(i) = getBuoyancyCenterVelocity(state, i);
    }
    if(frame == FrameType::GLOBAL){
        // 计算旋转带来的附加速度分量
        Vec34 feetPos = getBuoyancyCenter2BPositions(state, FrameType::BODY);
        feetVel += skew(state.getGyro()) * feetPos;// 角速度叉乘位置项

        // 转换到全局坐标系
        return state.getRotMat() * feetVel;
    }
    else if((frame == FrameType::BODY) || (frame == FrameType::HIP)){
        return feetVel;// 直接返回机体/髋关节坐标系速度
    }
    else{
        std::cout << "[ERROR] Frame error of function getFeet2BVelocities" << std::endl;
        exit(-1);
    }   
}

Vec34 QuadrupedRobot::getMassCenter2BVelocities(LowlevelState_XM &state, FrameType frame){
    // 定义一个3x4的矩阵，用于存储四条腿足端在指定坐标系下的速度
    Vec34 feetVel;
    // 获取各足端在机体坐标系中的基础速度
    for(int i(0); i<4; ++i){
        feetVel.col(i) = getMassCenterVelocity(state, i);
    }
    if(frame == FrameType::GLOBAL){
        // 计算旋转带来的附加速度分量
        Vec34 feetPos = getMassCenter2BPositions(state, FrameType::BODY);
        feetVel += skew(state.getGyro()) * feetPos;// 角速度叉乘位置项

        // 转换到全局坐标系
        return state.getRotMat() * feetVel;
    }
    else if((frame == FrameType::BODY) || (frame == FrameType::HIP)){
        return feetVel;// 直接返回机体/髋关节坐标系速度
    }
    else{
        std::cout << "[ERROR] Frame error of function getFeet2BVelocities" << std::endl;
        exit(-1);
    }  
}

Vec34 QuadrupedRobot::getFeet2BVelocities(LowlevelState &state, FrameType frame){
    Vec34 feetVel;
    for(int i(0); i<4; ++i){
        feetVel.col(i) = getFootVelocity(state, i);
    }

    if(frame == FrameType::GLOBAL){
        Vec34 feetPos = getFeet2BPositions(state, FrameType::BODY);
        feetVel += skew(state.getGyro()) * feetPos;
        return state.getRotMat() * feetVel;
    }
    else if((frame == FrameType::BODY) || (frame == FrameType::HIP)){
        return feetVel;
    }
    else{
        std::cout << "[ERROR] Frame error of function getFeet2BVelocities" << std::endl;
        exit(-1);
    }   
}

/**
 * 获取指定腿的雅可比矩阵
 * @param state 包含关节角度信息的底层状态
 * @param legID 腿的索引（0 - 3 对应四条腿）
 * @return 3x3 的雅可比矩阵，描述关节空间到操作空间的映射关系
 * 
 * 实现原理：
 * 1. 从底层状态中获取所有关节的角度
 * 2. 提取指定腿的关节角度
 * 3. 调用对应腿对象的 calcJaco 方法计算雅可比矩阵
 */
Mat3 QuadrupedRobot::getJaco_XM(LowlevelState_XM &state, int legID){
    Mat3 Jaco;
    if(legID == 0 || legID == 2){
        Jaco = _Legs[legID]->calcJaco_F(state.getQ().col(legID));
    }else{
        Jaco = _Legs[legID]->calcJaco_B(state.getQ().col(legID));
    }
    return Jaco;
}

Mat3 QuadrupedRobot::getJaco(LowlevelState &state, int legID){
    return _Legs[legID]->calcJaco(state.getQ().col(legID));
}

/**
 * A1型号机器人模型构造函数
 * 功能：初始化机器人物理参数和运动限制
 *  - 定义四条腿的安装位置（机体坐标系）
 *  - 设置标准站立姿态
 *  - 配置运动速度限制
 */
A1Robot::A1Robot(){
    Vec3 defaultMassQ(0.1, 0, 0.05);    // 示例质心偏移参数
    Vec3 defaultBuoyancyQ(0, 0, 0.08);  // 示例浮心偏移参数

    // 初始化腿部安装位置（单位：米）
    // 参数格式：Vec3(前后位置, 左右位置, 高度)
    _Legs[0] = new A1Leg(0, Vec3(-0.1881,  0.04675, 0),Vec3( 0.1881, -0.04675, 0), defaultMassQ, defaultBuoyancyQ);
    _Legs[1] = new A1Leg(1, Vec3(-0.1881,  0.04675, 0),Vec3( 0.1881, -0.04675, 0), defaultMassQ, defaultBuoyancyQ);
    _Legs[2] = new A1Leg(2, Vec3(-0.1881,  0.04675, 0),Vec3( 0.1881, -0.04675, 0), defaultMassQ, defaultBuoyancyQ);
    _Legs[3] = new A1Leg(3, Vec3(-0.1881,  0.04675, 0),Vec3( 0.1881, -0.04675, 0), defaultMassQ, defaultBuoyancyQ);


    // 标准站立姿态足端位置（3行x4列矩阵）
    // 第一行：X轴坐标（前后方向，单位：米）
    // 第二行：Y轴坐标（左右方向，单位：米） 
    // 第三行：Z轴坐标（高度方向，单位：米）
    _feetPosNormalStand <<  0.1805,  0.1805, -0.1805, -0.1805, 
                           -0.1308,  0.1308, -0.1308,  0.1308,
                           -0.3180, -0.3180, -0.3180, -0.3180;

    // 设置机器人运动速度限制（单位：米/秒 和 弧度/秒）
    _robVelLimitX << -0.4, 0.4;// 前后速度限制 [-0.4, 0.4] m/s
    _robVelLimitY << -0.3, 0.3;// 横向速度限制 [-0.3, 0.3] m/s
    _robVelLimitYaw << -0.5, 0.5;// 偏航角速度限制 [-0.5, 0.5] rad/s

#ifdef COMPILE_WITH_REAL_ROBOT
    _mass = 12.5;
    _pcb << 0.01, 0.0, 0.0;
    _Ib = Vec3(0.132, 0.3475, 0.3775).asDiagonal();
#endif  // COMPILE_WITH_REAL_ROBOT

#ifdef COMPILE_WITH_SIMULATION
    _mass = 13.4;
    _pcb << 0.0, 0.0, 0.0;
    _Ib = Vec3(0.132, 0.3475, 0.3775).asDiagonal();
#endif  // COMPILE_WITH_SIMULATION
}

Go1Robot::Go1Robot(){
    Vec3 defaultMassQ(0.1, 0, 0.05);    // 示例质心偏移参数
    Vec3 defaultBuoyancyQ(0, 0, 0.08);  // 示例浮心偏移参数

    _Legs[0] = new Go1Leg(0, Vec3(-0.1881,  0.04675, 0),Vec3( 0.1881, -0.04675, 0), defaultMassQ, defaultBuoyancyQ);
    _Legs[1] = new Go1Leg(1, Vec3(-0.1881,  0.04675, 0),Vec3( 0.1881, -0.04675, 0), defaultMassQ, defaultBuoyancyQ);
    _Legs[2] = new Go1Leg(2, Vec3(-0.1881,  0.04675, 0),Vec3( 0.1881, -0.04675, 0), defaultMassQ, defaultBuoyancyQ);
    _Legs[3] = new Go1Leg(3, Vec3(-0.1881,  0.04675, 0),Vec3( 0.1881, -0.04675, 0), defaultMassQ, defaultBuoyancyQ);

    _feetPosNormalStand <<  0.1881,  0.1881, -0.1881, -0.1881,
                           -0.1300,  0.1300, -0.1300,  0.1300,
                           -0.3200, -0.3200, -0.3200, -0.3200;

    _robVelLimitX << -0.4, 0.4;
    _robVelLimitY << -0.3, 0.3;
    _robVelLimitYaw << -0.5, 0.5;


#ifdef COMPILE_WITH_REAL_ROBOT
    _mass = 10.5;
    _pcb << 0.04, 0.0, 0.0;
    _Ib = Vec3(0.0792, 0.2085, 0.2265).asDiagonal();
#endif  // COMPILE_WITH_REAL_ROBOT

#ifdef COMPILE_WITH_SIMULATION
    _mass = 12.0;
    _pcb << 0.0, 0.0, 0.0;
    _Ib = Vec3(0.0792, 0.2085, 0.2265).asDiagonal();
#endif  // COMPILE_WITH_SIMULATION
}

XMTurtle::XMTurtle(){

    Vec3 defaultMassQ(0, 0, 0);    // 示例质心偏移参数
    Vec3 defaultBuoyancyQ(0, 0, 0);  // 示例浮心偏移参数

    _Legs[0] = new XMturtle(0, Vec3( 0.1881, -0.04675, 0),Vec3( 0.1881, -0.04675, 0), defaultMassQ, defaultBuoyancyQ);
    _Legs[1] = new XMturtle(1, Vec3( 0.1881,  0.04675, 0),Vec3( 0.1881, -0.04675, 0), defaultMassQ, defaultBuoyancyQ);
    _Legs[2] = new XMturtle(2, Vec3(-0.1881, -0.04675, 0),Vec3( 0.1881, -0.04675, 0), defaultMassQ, defaultBuoyancyQ);
    _Legs[3] = new XMturtle(3, Vec3(-0.1881,  0.04675, 0),Vec3( 0.1881, -0.04675, 0), defaultMassQ, defaultBuoyancyQ);

    _feetPosNormalStand <<  0.1881,  0.1881, -0.1881, -0.1881,
                           -0.1300,  0.1300, -0.1300,  0.1300,
                           -0.3200, -0.3200, -0.3200, -0.3200;

    _robVelLimitX << -0.4, 0.4;
    _robVelLimitY << -0.3, 0.3;
    _robVelLimitYaw << -0.5, 0.5;

#ifdef COMPILE_WITH_REAL_ROBOT
    _mass = 10.5;
    _pcb << 0.04, 0.0, 0.0;
    _Ib = Vec3(0.0792, 0.2085, 0.2265).asDiagonal();
#endif  // COMPILE_WITH_REAL_ROBOT

#ifdef COMPILE_WITH_SIMULATION
    _mass = 12.0;
    _pcb << 0.0, 0.0, 0.0;
    _Ib = Vec3(0.0792, 0.2085, 0.2265).asDiagonal();
#endif  // COMPILE_WITH_SIMULATION
}
