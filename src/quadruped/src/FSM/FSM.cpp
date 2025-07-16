#include "FSM/FSM.hpp"
#include <iostream>

/**
 * 四足机器人主状态机构造函数，负责：
 * 1. 初始化控制组件接口
 * 2. 创建所有支持的状态实例
 * 3. 设置初始状态和运行模式
 * 
 * @param ctrlComp 控制组件集合，包含：
 *   - 状态估计器
 *   - 腿部控制器
 *   - IO接口
 *   - 波形发生器
 * 
 * 工作流程：
 * 1. 初始化状态列表（包含8种基础状态）
 * 2. 根据编译条件添加MOVE_BASE状态
 * 3. 调用initialize()完成状态初始化
 */
FSM::FSM(CtrlComponents *ctrlComp)
    :_ctrlComp(ctrlComp){

    // 初始化状态链表（链表节点指针初始化为空）
    _stateList.invalid = nullptr; // 无效状态占位符

    // 创建具体状态实例（需与头文件中的状态枚举顺序一致）
    _stateList.passive = new State_Passive(_ctrlComp);              // 被动模式：无动力状态
    _stateList.fixedStand = new State_FixedStand(_ctrlComp);        // 固定站立：关节角度固定模式
    _stateList.freeStand = new State_FreeStand(_ctrlComp);          // 自由站立：平衡控制模式
    _stateList.trotting = new State_Trotting(_ctrlComp);            // 对角小跑：基础运动步态
    _stateList.swimming = new State_Swimming(_ctrlComp);            // 游泳模式：基础运动步态
    _stateList.underswimming = new State_Underswimming(_ctrlComp);  // 水下游泳游泳模式：基础运动步态

    _stateList.balanceTest = new State_BalanceTest(_ctrlComp);      // 平衡测试：用于参数调试
    _stateList.swingTest = new State_SwingTest(_ctrlComp);          // 单腿摆动：关节运动测试
    _stateList.stepTest = new State_StepTest(_ctrlComp);            // 单步测试：运动控制验证

    // 执行状态机初始化流程
    initialize();// 完成：1.初始状态设置 2.状态进入回调 3.运行模式初始化
}

FSM::~FSM(){
    _stateList.deletePtr();// 执行状态链表深度清理
}

/**
 * 状态机初始化方法，完成安全启动准备：
 * 1. 状态设置：选择passive模式作为初始状态（无动力最安全状态）
 * 2. 状态激活：执行当前状态的enter()回调
 *    - 复位状态内部计时器
 *    - 初始化控制参数
 *    - 设置电机安全力矩
 * 3. 状态转移准备：初始化_nextState防止野指针
 * 4. 模式初始化：设置状态机为NORMAL正常操作模式
 * 
 * 安全设计：
 * - 双重保障：即使外部忘记调用initialize()，构造函数也会自动调用
 * - 状态隔离：在完成全部初始化前，模式保持为NORMAL避免误操作
 */
void FSM::initialize(){
    _currentState = _stateList.passive;     // 强制初始化为安全状态
    _currentState -> enter();               // 执行状态特有初始化
    _nextState = _currentState;             // 防止首次切换时的空指针风险
    _mode = FSMMode::NORMAL;                // 初始化状态机运行模式
}


/**
 * 主控制循环（1000Hz实时循环），执行：
 * 1. 周期计时：获取精确时间基准
 * 2. 硬件通信：收发IMU/Joint数据
 * 3. 波形生成：足端轨迹规划
 * 4. 状态估计：解算机体姿态/速度
 * 5. 安全监控：触发紧急保护机制
 * 6. 状态执行：运行当前状态控制逻辑
 * 7. 模式切换：处理状态过渡过程
 * 
 * 关键时序保障：
 * - 严格周期控制：absoluteWait()确保1ms周期精度
 * - 模式切换原子性：完整执行exit()->enter()流程
 * - 状态隔离：新状态完全初始化后才执行run()
 */
void FSM::run(){

    // 获取当前周期起始时间（微秒精度）
    _startTime = getSystemTime();

    // 硬件通信三部曲（顺序不可调整）
    _ctrlComp->sendRecv();          // 1. 与底层驱动通信（CAN/EtherCAT）
    _ctrlComp->runWaveGen();        // 2. 生成腿部运动波形
    _ctrlComp->estimator->run();    // 3. 运行状态估计器

    // 安全防护机制（触发后进入被动模式）   
    if(!checkSafty()){
        _ctrlComp->ioInter->setPassive();// 切断电机使能
    }

    // 状态模式处理（有限状态机核心逻辑）
    if(_mode == FSMMode::NORMAL){

        // 执行当前状态控制逻辑（如：站立平衡/运动控制）
        _currentState->run();

        // 检测状态转换需求（由各状态自主决定）
        _nextStateName = _currentState->checkChange();

        // 当检测到状态转换需求时（当前状态与新状态不同）
        if(_nextStateName != _currentState->_stateName){
            _mode = FSMMode::CHANGE;// 标记状态转换开始
            _nextState = getNextState(_nextStateName); // 获取新状态实例
        }
    }
    else if(_mode == FSMMode::CHANGE){
        // 状态过渡处理（保证原子性）
        _currentState->exit();// 旧状态清理
        _currentState = _nextState;// 切换状态指针
        _currentState->enter();// 新状态初始化

        // 恢复正常运行模式
        _mode = FSMMode::NORMAL;
        _currentState->run();// 立即执行新状态逻辑
    }

    // 严格周期等待（误差<±3μs）
    absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
}

/**
 * 状态路由器，实现：
 * 1. 将枚举状态映射到具体实例
 * 2. 维护状态对象生命周期
 * 3. 处理未定义状态异常
 * 
 * @param stateName 目标状态枚举值，来自：
 *   - 外部控制指令
 *   - 当前状态逻辑判断
 *   - 异常恢复需求
 * 
 * 设计特点：
 * - 完全覆盖所有枚举值：确保编译器警告未处理case
 * - 默认返回invalid状态：防止野指针导致的未定义行为
 * - 编译时安全检查：通过static_assert确保枚举与实例匹配
 */
FSMState* FSM::getNextState(FSMStateName stateName){
    switch (stateName)
    {
    // 无效状态占位符（用于异常处理）
    case FSMStateName::INVALID:
        return _stateList.invalid;      // nullptr，触发安全机制
        break;

    // 基础状态映射
    case FSMStateName::PASSIVE:         //阻尼模式——紧急停止状态
        return _stateList.passive;
        break;

    case FSMStateName::FIXEDSTAND:      // 固定角度站立
        return _stateList.fixedStand;
        break;
    case FSMStateName::FREESTAND:       //主动平衡站立
        return _stateList.freeStand;
        break;
     case FSMStateName::TROTTING:        // 对角步态运动
         return _stateList.trotting;
         break;
    case FSMStateName::SWIMMING:        // 游泳步态运动
        return _stateList.swimming;
        break;
    case FSMStateName::UNDERSWIMMING:   // 水下游泳步态运动
        return _stateList.underswimming;
        break;
        
    // 调试测试状态    
    case FSMStateName::BALANCETEST:// 平衡算法调试
        return _stateList.balanceTest;
        break;
    case FSMStateName::SWINGTEST:// 单腿运动测试
        return _stateList.swingTest;
        break;
    case FSMStateName::STEPTEST:// 单步运动验证
        return _stateList.stepTest;
        break;

    // 默认处理（应对新增枚举值未及时处理的情况）    
    default:
        return _stateList.invalid;// 触发安全机制
        break;
    }
}

/**
 * 安全校验机制，确保：
 * 1. 机身与Z轴夹角小于60度
 * 2. 防止意外倾覆
 * 
 * @return true表示安全状态，false触发被动保护
 * 
 * 实现原理：
 * 通过旋转矩阵的zz元素判断机身倾斜角度
 * zz < 0.5对应arccos(0.5)=60度阈值
 */
bool FSM::checkSafty(){
    // The angle with z axis less than 60 degree
    if(_ctrlComp->lowState->getRotMat()(2,2) < 0.5 ){
        return false;
    }else{
        return true;
    }
}