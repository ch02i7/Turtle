
#include "Gait/GaitGenerator.h"

GaitGenerator::GaitGenerator(CtrlComponents *ctrlComp)
              : _waveG(ctrlComp->waveGen),          // 步态周期生成器（获取摆动/支撑相时间）
                _est(ctrlComp->estimator),          // 状态估计器（获取机器人位姿信息）
                _phase(ctrlComp->phase),            // 步态相位生成器（获取各腿相位值）
                _contact(ctrlComp->contact),        // 足端接触状态检测器
                _robModel(ctrlComp->robotModel),    // 机器人运动学模型
                _state(ctrlComp->lowState)          // 底层传感器数据
                 {
    _feetCal = new FeetEndCal(ctrlComp);            // 创建足端轨迹计算器实例
    _firstRun = true;                               // 首次运行标志（触发初始位置获取）
}

GaitGenerator::~GaitGenerator(){
}

/**
 * 设置步态生成关键参数
 * @param vxyGoalGlobal 全局坐标系下的X-Y平面速度目标 [vx, vy] (m/s)
 * @param dYawGoal 期望偏航角速度 (rad/s)
 * @param gaitHeight 摆动相足端抬升高度 (m)
 * 
 * 参数作用：
 * - vxyGoalGlobal：控制机器人前进/后退及横向移动速度
 * - dYawGoal：控制机器人绕Z轴旋转速度（左/右转向）
 * - gaitHeight：调整足端在空中的抛物线轨迹高度
 */
void GaitGenerator::setGait(Vec2 vxyGoalGlobal, float dYawGoal, float gaitHeight){
    _vxyGoal = vxyGoalGlobal;
    _dYawGoal = dYawGoal;
    _gaitHeight = gaitHeight;
}

void GaitGenerator::setGait_swimming(Vec2 vxyGoalGlobal, float dYawGoal, float dPitchGoal){
    _vxyGoal = vxyGoalGlobal;
    _dYawGoal = dYawGoal;
    _dPitchGoal = dPitchGoal;

}

/**
 * 重置步态生成器状态
 * 功能：
 * 1. 标记首次运行标志，使得下次运行时会重新初始化起始位置
 * 2. 清零水平面速度控制目标
 * 
 * 触发场景：
 * - 步态模式切换时
 * - 紧急停止后恢复时
 */
void GaitGenerator::restart(){
    _firstRun = true;// 触发起始位置重新初始化
    _vxyGoal.setZero(); // 重置x-y平面速度目标为0
}

/**
 * 主步态生成函数（实时更新足端轨迹）
 * @param feetPos 输出：四足目标位置矩阵（全局坐标系）
 * @param feetVel 输出：四足目标速度矩阵（全局坐标系）
 * 
 * 实现流程：
 * 1. 首次运行时初始化起始位置
 * 2. 遍历四条腿，根据接触状态生成轨迹：
 *    - 支撑腿：保持当前位置，速度清零
 *    - 摆动腿：计算摆线轨迹的目标位置和速度
 * 3. 保存当前相位状态供下次迭代使用
 */
void GaitGenerator::run(Vec34 &feetPos, Vec34 &feetVel){
    if(_firstRun){// 首次运行初始化
        _startP = _est->getFeetPos();// 获取当前足端位置作为起始点
        _firstRun = false;
    }

    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 1){// 支撑腿处理
            if((*_phase)(i) < 0.5){// 相位前半段更新起始位置
                _startP.col(i) = _est->getFootPos(i);// 实时更新支撑腿起始位置
            }
            feetPos.col(i) = _startP.col(i);// 保持当前位置
            feetVel.col(i).setZero(); // 支撑腿速度为零
        }
        else{// 摆动腿处理
            _endP.col(i) = _feetCal->calFootPos(i, _vxyGoal, _dYawGoal, (*_phase)(i));// 计算摆动终点

            feetPos.col(i) = getFootPos(i);// 获取摆线轨迹位置
            feetVel.col(i) = getFootVel(i);// 获取摆线轨迹速度
        }
    }
    _pastP = feetPos;// 保存当前足端位置
    _phasePast = *_phase; // 保存当前相位信息
}

void GaitGenerator::run_swimming_F(Vec34 &feetPos, Vec34 &feetVel){
    if(_firstRun){// 首次运行初始化
        _startP = _est->getBuoyancyCenterPos();// 获取当前足端位置作为起始点
        _firstRun = false;
    }
    for(int i(0); i<2; ++i){
        _endP.col(i) = _feetCal->calBuoyancyPos(i, _vxyGoal, _dYawGoal,_dPitchGoal, (*_phase)(i));// 计算摆动终点
        feetPos.col(i) = getFootPos_swimming_F(i);// 获取摆线轨迹位置
        feetVel.col(i) = getFootVel_swimming_F(i);// 获取摆线轨迹速度
    }
    _pastP = feetPos;// 保存当前足端位置
    _phasePast = *_phase; // 保存当前相位信息

}

void GaitGenerator::run_swimming_B(Vec34 &feetPos, Vec34 &feetVel){
    if(_firstRun){// 首次运行初始化
        _startP = _est->getBuoyancyCenterPos();// 获取当前足端位置作为起始点
        _firstRun = false;
    }
    for(int i(2); i<4; ++i){
        _endP.col(i) = _feetCal->calBuoyancyPos_B(i, _vxyGoal, _dYawGoal,_dPitchGoal, (*_phase)(i));// 计算摆动终点
        feetPos.col(i) = getFootPos_swimming_B(i);// 获取摆线轨迹位置
        feetVel.col(i) = getFootVel_swimming_B(i);// 获取摆线轨迹速度
    }
    _pastP = feetPos;// 保存当前足端位置
    _phasePast = *_phase; // 保存当前相位信息

}
/**
 * 计算单腿摆线轨迹位置
 * @param i 腿的索引（0-3）
 * @return 三维位置向量 [x,y,z]^T (m)
 * 
 * 轨迹生成原理：
 * X/Y方向：使用摆线函数实现平滑的起始-结束过渡
 * Z方向：抛物线轨迹实现足端抬升
 * 
 * 数学公式：
 * X/Y轴：cycloidXYPosition = (end-start)*(2πφ - sin(2πφ))/(2π) + start
 * Z轴：cycloidZPosition = h*(1 - cos(2πφ))/2 + start_z
 */
Vec3 GaitGenerator::getFootPos(int i){
    Vec3 footPos;

    // X/Y轴摆线轨迹（相位从0到1完成起始点到终点的移动）
    footPos(0) = cycloidXYPosition(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    footPos(1) = cycloidXYPosition(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));

    // Z轴抛物线轨迹（实现足端抬升）
    footPos(2) =  cycloidZPosition(_startP.col(i)(2), _gaitHeight, (*_phase)(i));
    
    return footPos;
}

Vec3 GaitGenerator::getFootPos_swimming_F(int i){
    Vec3 footPos;

    // X/Y轴摆线轨迹（相位从0到1完成起始点到终点的移动）
    footPos(0) = cycloidXYPosition(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    footPos(1) = cycloidXYPosition(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));

    return footPos;
}

Vec3 GaitGenerator::getFootPos_swimming_B(int i){
    Vec3 footPos;

    // X/Y轴摆线轨迹（相位从0到1完成起始点到终点的移动）
    footPos(0) = cycloidXYPosition(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    footPos(1) = cycloidXYPosition(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));
    
    return footPos;
}


/**
 * 计算单腿摆线轨迹速度
 * @param i 腿的索引（0-3）
 * @return 三维速度向量 [vx,vy,vz]^T (m/s)
 * 
 * 速度计算原理：
 * 对摆线位置函数求导得到瞬时速度
 * 相位参数φ由步态周期发生器提供
 * 
 * 数学公式：
 * X/Y轴：cycloidXYVelocity = (end-start)*(1 - cos(2πφ)) / T_swing
 * Z轴：cycloidZVelocity = hπ*sin(2πφ) / T_swing
 */
Vec3 GaitGenerator::getFootVel(int i){
    Vec3 footVel;

    // X/Y轴速度计算（摆线导数）
    footVel(0) = cycloidXYVelocity(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    footVel(1) = cycloidXYVelocity(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));
    // Z轴速度计算（抛物线导数）
    footVel(2) =  cycloidZVelocity(_gaitHeight, (*_phase)(i));

    return footVel;
}

Vec3 GaitGenerator::getFootVel_swimming_F(int i){
    Vec3 footVel;

    // X/Y轴速度计算（摆线导数）
    footVel(0) = cycloidXYVelocity(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    footVel(1) = cycloidXYVelocity(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));

    return footVel;
}

Vec3 GaitGenerator::getFootVel_swimming_B(int i){
    Vec3 footVel;

    // X/Y轴速度计算（摆线导数）
    footVel(0) = cycloidXYVelocity(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    footVel(1) = cycloidXYVelocity(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));

    return footVel;
}

/**
 * 计算XY平面摆线轨迹位置 
 * @param start 起始位置（m）
 * @param end 目标位置（m）
 * @param phase 当前相位 [0,1]
 * @return 平滑过渡的位置值
 * 
 * 摆线方程原理：
 * 实现起始点到终点的平滑加速度运动，避免速度突变
 * 公式推导：s = (end - start)*(θ - sinθ)/(2π) + start, θ=2π*phase
 */
float GaitGenerator::cycloidXYPosition(float start, float end, float phase){
    float phasePI = 2 * M_PI * phase;
    return (end - start)*(phasePI - sin(phasePI))/(2*M_PI) + start;
}

/**
 * 计算XY平面摆线轨迹速度
 * @param start 起始位置（m）
 * @param end 目标位置（m） 
 * @param phase 当前相位 [0,1]
 * @return 瞬时速度值（m/s）
 * 
 * 速度公式推导：
 * 对摆线位置方程求导，v = ds/dphase * (1/Tswing)
 * 保证在相位起点和终点时速度为零
 */
float GaitGenerator::cycloidXYVelocity(float start, float end, float phase){
    float phasePI = 2 * M_PI * phase;
    return (end - start)*(1 - cos(phasePI)) / _waveG->getTswing();
}

/**
 * 计算Z轴抛物线轨迹位置
 * @param start Z轴起始高度（通常为0）
 * @param h 期望抬升高度（m）
 * @param phase 当前相位 [0,1] 
 * @return 垂直方向位置值（m）
 * 
 * 轨迹特性：
 * - 相位0和1时高度为0（接触地面）
 * - 相位0.5时达到最大高度h
 */
float GaitGenerator::cycloidZPosition(float start, float h, float phase){
    float phasePI = 2 * M_PI * phase;
    return h*(1 - cos(phasePI))/2 + start;
}

/**
 * 计算Z轴抛物线轨迹速度 
 * @param h 抬升高度（m）
 * @param phase 当前相位 [0,1]
 * @return 垂直方向速度值（m/s）
 * 
 * 速度特性：
 * - 相位0.25时达到最大上升速度
 * - 相位0.75时达到最大下降速度
 * - 相位0和1时速度为零
 */
float GaitGenerator::cycloidZVelocity(float h, float phase){
    float phasePI = 2 * M_PI * phase;
    return h*M_PI * sin(phasePI) / _waveG->getTswing();
}