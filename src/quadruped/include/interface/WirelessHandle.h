
#ifndef WIRELESSHANDLE_H
#define WIRELESSHANDLE_H

#include "message/PS_joystick.h"
#include "interface/CmdPanel.h"
#include "message/LowlevelState.h"
#include "sensor_msgs/msg/joy.hpp"  // 添加Joy消息头文件

class WirelessHandle : public CmdPanel{
public:
    WirelessHandle();
    ~WirelessHandle(){}
    void receiveHandle(LowlevelState *lowState);
    void parseJoyData(const sensor_msgs::msg::Joy::SharedPtr msg);

    UserCommand getCmd() const { return userCmd; }
    UserValue getValue() const { return userValue; }
private:
    xRockerBtnDataStruct _keyData;
};

#endif  // WIRELESSHANDLE_H