#include "interface/WirelessHandle.h"
#include "common/mathTools.h"
#include <string.h>
#include <stdio.h>
#include "message/LowlevelState.h"

WirelessHandle::WirelessHandle(){
}

void WirelessHandle::receiveHandle(LowlevelState *lowState){
 
    if(((int)_keyData.btn.components.L2 == 1) && 
       ((int)_keyData.btn.components.B  == 1)){
        userCmd = UserCommand::L2_B;
    }
    else if(((int)_keyData.btn.components.L2 == 1) && 
            ((int)_keyData.btn.components.A  == 1)){
        userCmd = UserCommand::L2_A;
    }
    else if(((int)_keyData.btn.components.L2 == 1) && 
            ((int)_keyData.btn.components.X  == 1)){
        userCmd = UserCommand::L2_X;
    }

    else if(((int)_keyData.btn.components.L1 == 1) && 
            ((int)_keyData.btn.components.X  == 1)){
        userCmd = UserCommand::L1_X;
    }
    else if(((int)_keyData.btn.components.L1 == 1) && 
            ((int)_keyData.btn.components.A  == 1)){
        userCmd = UserCommand::L1_A;
    }
    else if(((int)_keyData.btn.components.L1 == 1) && 
            ((int)_keyData.btn.components.Y  == 1)){
        userCmd = UserCommand::L1_Y;
    }
    else if((int)_keyData.btn.components.start == 1){
        userCmd = UserCommand::START;
    }

    userValue.L2 = killZeroOffset(_keyData.L2, 0.08);
    userValue.lx = killZeroOffset(_keyData.lx, 0.08);
    userValue.ly = killZeroOffset(_keyData.ly, 0.08);
    userValue.rx = killZeroOffset(_keyData.rx, 0.08);
    userValue.ry = killZeroOffset(_keyData.ry, 0.08);
}


void WirelessHandle::parseJoyData(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // 将ROS2的joy消息转换为内部_keyData格式
    _keyData.btn.components.L2 = msg->buttons[5];  // 假设L2对应按钮索引5
    _keyData.btn.components.B = msg->buttons[1];   // B按钮索引1
    _keyData.btn.components.A = msg->buttons[0];  // A按钮索引0
    _keyData.btn.components.X = msg->buttons[2];   // X按钮索引2
    _keyData.btn.components.Y = msg->buttons[3];   // Y按钮索引3
    _keyData.btn.components.L1 = msg->buttons[4];  // L1按钮索引4
    _keyData.btn.components.start = msg->buttons[9]; // START按钮索引9

    // 转换摇杆数据
    _keyData.lx = msg->axes[0];  // 左摇杆X轴
    _keyData.ly = msg->axes[1];  // 左摇杆Y轴
    _keyData.rx = msg->axes[2];  // 右摇杆X轴
    _keyData.ry = msg->axes[3];  // 右摇杆Y轴
    _keyData.L2 = msg->axes[5];  // L2扳机值
}

