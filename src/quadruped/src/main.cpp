#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "control/ControlFrame.h"
#include "control/CtrlComponents.h"
#include "sensor_msgs/msg/joy.hpp"
#include "interface/WirelessHandle.h"
#include <pthread.h>
#include "serial_xm/robot_connect.h"

#include "quadruped/msg/motor_cmd_msg.hpp"
#include "quadruped/msg/motor_state_msg.hpp"

class QuadrupedControlNode : public rclcpp::Node {
public:
    QuadrupedControlNode(rclcpp::NodeOptions options)  // 添加节点选项参数
        : Node("quadruped_controller", options) {     // 显式传递options
        _initComponents();    // 初始化控制算法组件       
        _setupTimers();       // 配置实时定时器
        _setupjoy();           // 手柄订阅器
        // _setupMotorCmdSub();   // 电机指令订阅器
    //    _setupStatePublisher();//电机状态发布器
    }  

private:

//电机状态发布器
void _setupStatePublisher() {
    _motorStatePub = create_publisher<quadruped::msg::MotorStateMsg>("motor_states", 10);
     // 100Hz发布频率
    _statePubTimer = create_wall_timer( 
        std::chrono::milliseconds(1000),
        [this]() {
            auto msg = quadruped::msg::MotorStateMsg();
            
            // 仅填充需要的参数
            for(int i = 0; i < 16; ++i) {
                const auto& motor = ctrlComp->lowState_xm->motorState[i];

                msg.motor_index.push_back(i);
                msg.q.push_back(motor.q);
                msg.dq.push_back(motor.dq);
                msg.ddq.push_back(motor.ddq);
                msg.tau_est.push_back(motor.tauEst);

            }
            _motorStatePub->publish(msg);
        });
}

// 电机指令订阅器
void _setupMotorCmdSub() {
    _motorCmdSub = create_subscription<quadruped::msg::MotorCmdMsg>(
        "motor_commands", 10,
        [this](const quadruped::msg::MotorCmdMsg::SharedPtr msg) {
        
                        // 新增空指针保护
            if(!ctrlComp || !ctrlComp->lowCmd) {
                RCLCPP_ERROR(get_logger(), "收到指令时控制组件未初始化！");
                return;
            }

            // 新增长度校验
            size_t cmd_size = msg->motor_index.size();
            if(cmd_size != msg->modes.size() || cmd_size != msg->q_targets.size()) {
                RCLCPP_WARN(get_logger(), "指令参数长度不一致 motor_index:%zu 其他参数长度:%zu,%zu",
                          cmd_size, msg->modes.size(), msg->q_targets.size());
                return;
            }
            // 遍历所有要修改的电机
            for(size_t i = 0; i < msg->motor_index.size(); ++i) {
                int idx = msg->motor_index[i];
                if(idx < 0 || idx >= 16) continue;  // 保护索引范围
                
                auto& cmd = ctrlComp->lowCmd_xm->motorCmd[idx];
                    cmd.mode = msg->modes[i];
                    cmd.q = msg->q_targets[i];
                    cmd.dq = msg->dq_targets[i];
                    cmd.tau = msg->tau_targets[i];
                    cmd.Kp = msg->kp_gains[i];
                    cmd.Kd = msg->kd_gains[i];
                }
        });
}

    //初始化手柄
    void _setupjoy(){
         // 初始化手柄处理器
        _wirelessHandle = std::make_unique<WirelessHandle>();

        // 订阅手柄话题
        _joySub = create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10,
        [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
            _wirelessHandle->parseJoyData(msg);
            _wirelessHandle->receiveHandle(ctrlComp->lowState);
            
            // 直接更新控制指令
            ctrlComp->lowState->userCmd = _wirelessHandle->getCmd();
            ctrlComp->lowState->userValue = _wirelessHandle->getValue();
            
// RCLCPP_INFO(this->get_logger(), 
//     "手柄数据 | LX:%.2f LY:%.2f RX:%.2f RY:%.2f 按键状态:%d",
//     ctrlComp->lowState->userValue.lx,
//     ctrlComp->lowState->userValue.ly,
//     ctrlComp->lowState->userValue.rx,
//     ctrlComp->lowState->userValue.ry,
//     static_cast<int>(_wirelessHandle->getCmd()));  // 直接转换枚举值为整型
         });
    }


    // 初始化控制算法组件
    void _initComponents() {

        // 传递lowCmd/lowState到控制组件
        ctrlComp = std::make_unique<CtrlComponents>();

        // 在节点中声明并获取参数
        this->declare_parameter<std::string>("serial_port", "/dev/ttyROBOT");
        this->declare_parameter<int>("baud_rate", 921600);
        std::string port;
        int baud;
        this->get_parameter("serial_port", port);
        this->get_parameter("baud_rate", baud);

        // 直接传递参数到robot构造函数
        _robot = std::make_unique<robot>(port, baud);
        ctrlComp->ioInter = _robot.get();

        ctrlComp->ctrlPlatform = CtrlPlatform::REALROBOT;  // 设置为真实机器人模式
        ctrlComp->dt = 0.01;  // 控制周期2ms（500Hz）
        
        _xm_robot = std::make_unique<XMTurtle>();
        ctrlComp->robotModel = _xm_robot.get();
        
        // 创建步态生成器（摆动周期0.45s，占空比0.5）
        auto wave_gen = std::make_unique<WaveGenerator>(0.45, 0.5, Vec4(0, 0.5, 0.5, 0));
        ctrlComp->waveGen = wave_gen.release();
        
        ctrlComp->geneObj();  // 生成控制对象
        ctrlFrame = std::make_unique<ControlFrame>(ctrlComp.get());  // 创建控制框架
    }

void _setupTimers() {
    _control_timer = create_wall_timer(
        std::chrono::microseconds(10000),  
        [this]() {
            ctrlFrame->run();
                RCLCPP_INFO(this->get_logger(), "串口状态: %s,当前FSM状态: %d", 
                    ctrlComp->lowState_xm->serial_tip.c_str(),
                    ctrlComp->lowState->userCmd
                );
        });
}
    // 添加以下成员声明
    std::unique_ptr<WirelessHandle> _wirelessHandle;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joySub;

    std::unique_ptr<CtrlComponents> ctrlComp;
    std::unique_ptr<ControlFrame> ctrlFrame;
    rclcpp::TimerBase::SharedPtr _control_timer;

    std::unique_ptr<robot> _robot;        // 新增机器人通信对象持有者
    std::unique_ptr<XMTurtle> _xm_robot; // 新增机器人模型持有者

    // 在QuadrupedControlNode类的private成员中添加订阅器
    rclcpp::Subscription<quadruped::msg::MotorCmdMsg>::SharedPtr _motorCmdSub;
    rclcpp::Publisher<quadruped::msg::MotorStateMsg>::SharedPtr _motorStatePub;
    rclcpp::TimerBase::SharedPtr _statePubTimer;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<QuadrupedControlNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}