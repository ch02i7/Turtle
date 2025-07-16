#ifndef FSM_H
#define FSM_H

/**
 * 状态机核心头文件，包含：
 * 1. 状态枚举定义
 * 2. 状态机主类声明
 * 3. 状态列表管理结构
 */
#include "FSM/FSMState.h"
#include "FSM/State_FixedStand.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FreeStand.h"
#include "FSM/State_Trotting.h"
#include "FSM/State_BalanceTest.h"
#include "FSM/State_SwingTest.h"
#include "FSM/State_StepTest.h"
#include "FSM/State_Swimming.h"
#include "State_Underswimming.h"

#include "common/enumClass.h"           // 状态枚举定义
#include "control/CtrlComponents.h"     // 控制组件集合


/**
 * 状态列表管理结构体，负责：
 * 1. 集中管理所有状态实例指针
 * 2. 提供统一的内存释放接口
 * 
 * 包含状态：
 * - passive   被动模式
 * - fixedStand 固定站立 
 * - freeStand 自由站立
 * - trotting  对角小跑
 * - swimming  游泳模式
 * - underswimming 水下游泳模式
 * - 调试状态组（balanceTest/swingTest/stepTest）
 * - moveBase 移动底盘（条件编译）
 */
struct FSMStateList{
    FSMState *invalid;                      // 无效状态占位符
    State_Passive *passive;                 // 被动无动力状态（阻尼状态）
    State_FixedStand *fixedStand;           // 关节固定站立
    State_FreeStand *freeStand;             // 主动平衡站立
    State_Trotting *trotting;               // 基础运动步态
    State_Swimming *swimming;               // 游泳模式
    State_Underswimming *underswimming;     // 水下游泳模式

    State_BalanceTest *balanceTest;         // 平衡算法调试
    State_SwingTest *swingTest;             // 单腿摆动测试 
    State_StepTest *stepTest;               // 单步运动验证


/**
* 统一内存释放方法，特性：
* 1. 自动跳过nullptr避免重复删除
* 2. 逆序删除防止依赖问题
* 3. 虚析构机制保证完全释放
 */
    void deletePtr(){
        // 按创建顺序逆序删除
        delete invalid;
        delete passive;
        delete fixedStand;
        delete freeStand;
        delete trotting;
        delete swimming;
        delete underswimming;

        delete balanceTest;
        delete swingTest;
        delete stepTest;
    }
};


/**
 * 四足机器人主状态机类，功能：
 * 1. 管理状态生命周期
 * 2. 处理状态转换逻辑
 * 3. 保障系统安全运行
 * 
 * 接口说明：
 * - initialize() 安全初始化
 * - run()       主控制循环
 * - checkSafty() 安全校验
 */
class FSM{
public:
    /**
     * 状态机构造函数
     * @param ctrlComp 控制组件集合指针，包含：
     *   - 状态估计器
     *   - 腿部控制器
     *   - 硬件接口
     *   - 参数配置
     */
    FSM(CtrlComponents *ctrlComp);

    ~FSM();                             // 资源清理
    void initialize();                  // 安全初始化
    void run();                         // 主控制循环

private:
    FSMState* getNextState(FSMStateName stateName);     // 状态路由
    bool checkSafty();                                  // 安全校验


    // 状态机核心组件    
    CtrlComponents *_ctrlComp;          // 控制组件集合（生命周期由外部管理）
    FSMState *_currentState;            // 当前活跃状态
    FSMState *_nextState;               // 待切换状态
    FSMStateName _nextStateName;        // 下一状态标识
    FSMStateList _stateList;            // 状态实例容器
    FSMMode _mode;                      // 运行模式
    long long _startTime;               // 周期计时基准
    int count;                          // 调试计数器
};


#endif  // FSM_H