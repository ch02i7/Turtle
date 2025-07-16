#include "serial_driver/serial_driver.hpp"
#include "message/LowlevelState.h"
#include "message/LowlevelCmd.h"
#include "serial_xm/robot_connect.h"
#include <thread>  // 需要添加头文件

using drivers::serial_driver::SerialDriver;
using drivers::serial_driver::SerialPortConfig;


robot::robot(const std::string& port, int baud) : 
    motor_serial_port(port),  // 初始化串口路径
    motor_seial_baud(baud),   // 初始化波特率
    io_ctx_(std::make_shared<drivers::common::IoContext>()),
    serial_motor_(std::make_unique<drivers::serial_driver::SerialDriver>(*io_ctx_))
{

  int i=0;
  for(auto& motor:motors) 
    {
      motor.name = "Motor_" + std::to_string(i);   
      motor.pos = 0.0f;
      motor.vel = 0.0f;
      motor.tor = 0.0f;
      motor.tor_set = 3.0f;
      motor.pos_set = 0.0f;
      motor.vel_set = 0.0f;
      motor.kp = 0.0f;
      motor.kd = 0.0f;
      motor.index=i;
      i++;
  }
  motors[0].type = "4340";
  motors[1].type = "4340";
  motors[2].type = "4340";
  motors[3].type =  "4340";              
  motors[4].type = "4340";
  motors[5].type = "4340";   
  motors[6].type = "4340";
  motors[7].type =  "4340";   
  motors[8].type = "4340";
  motors[9].type = "4340";      
  motors[10].type = "4340";  
  motors[11].type = "4340";  
  motors[12].type = "4340";  
  motors[13].type = "4340";            

  init_motor_serial();//初始化串口

}

robot::~robot()
{   
   for(int i=0;i<12;i++)
  {
    fresh_cmd_motor_data(0.0, 0.0, 0.0, 0.0, 0.0, i); //更新发给电机的参数、力矩等
  }

  if (serial_motor_ && serial_motor_->port()->is_open()) {
    serial_motor_->port()->close();
  }
}


void robot::init_motor_serial() 
{       
    try {
        const SerialPortConfig config(
        motor_seial_baud,
        drivers::serial_driver::FlowControl::NONE,    // 添加完整命名空间
        drivers::serial_driver::Parity::NONE,        // 添加完整命名空间
        drivers::serial_driver::StopBits::ONE        // 添加完整命名空间
        );
        
        serial_motor_->init_port(motor_serial_port, config);
        serial_motor_->port()->open();
        
    }       
    catch (const std::exception &e) {

    }
}


void robot::get_motor_data_thread(const LowlevelCmd *cmd, LowlevelState *state)
{   
    // 电机数据接收线程主循环
    while (!stop_thread_) {    
        // 使用互斥锁保护共享数据
        std::lock_guard<std::mutex> lock(motor_mutex_);
        std::vector<uint8_t> buffer(256);  // 创建接收缓冲区（256字节）
        
        try {
            // 批量读取串口数据
            static int count = 0;  // 数据包字节计数器
            const size_t bytes_read = serial_motor_->port()->receive(buffer);

            // 修正后的数据转换逻辑
            if (bytes_read > 0) {
                // 将二进制数据转换为十六进制字符串
                std::stringstream ss;
                for (size_t i = 0; i < bytes_read; ++i) {
                    ss << std::hex << std::setw(2) << std::setfill('0') 
                       << static_cast<int>(buffer[i]) << " ";
                }
                state->serial_tip = ss.str();  // 存储原始数据字符串
            }else{
              state->serial_tip = "无数据";  // 存储原始数据字符串
            }

        //     // 逐个字节处理接收数据
        //     for(size_t i=0; i<bytes_read; ++i) {
        //         Receive_Data.rx[count] = buffer[i];  // 存储当前字节
        //         Receive_Data.Frame_Header = Receive_Data.rx[0];  // 解析帧头

        //         // 数据包起始检测逻辑
        //         if(buffer[i] == FRAME_HEADER || count>0) {  // 匹配帧头或正在接收数据包
        //             count++;
        //         } else {
        //             count=0;  // 重置计数器
        //         }

        //         // 完整数据包接收完成
        //         if(count == RECEIVE_DATA_SIZE) {
        //             count=0;  // 重置计数器
        //             uint8_t check = Check_Sum(RECEIVE_DATA_SIZE-1,READ_DATA_CHECK);  // 计算校验和

        //             // 校验通过处理数据
        //             if(check == Receive_Data.rx[RECEIVE_DATA_SIZE-1]) {
        //                 // 解析12个电机的反馈数据
        //                 for(int i=0;i<12;i++) {
        //                     dm4340_fbdata(motors[i],&Receive_Data.rx[1+i*5]);  // 每个电机数据占5字节
        //                     // 保持原始索引映射（当前未使用，可后续扩展）
        //                     int state_index = i;
        //                 }
        //             }
        //         }
        //     }
         }
        catch (const std::exception &e) {
            // 异常处理（当前为空，可添加错误日志）
                state->serial_tip = "接收异常: " + std::string(e.what()); // 异常信息记录
        }
    }
}

void robot::send_motor_data(const LowlevelCmd *cmd, LowlevelState *state)
{   

  std::vector<uint8_t> tx_buffer;
  tx_buffer.reserve(39);

  Send_Data.tx[0] = (FRAME_HEADER);
  Send_Data.tx[1] = (0xA5); 
  for(int i=0; i<12; i++) 
  {            
    auto& motor = motors[i];
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

    pos_tmp = float_to_uint(motor.pos_set, P_MIN2, P_MAX2, 8); 
    vel_tmp = float_to_uint(motor.vel_set, V_MIN2, V_MAX2, 8); 
    tor_tmp = float_to_uint(motor.tor_set, T_MIN2, T_MAX2, 8); 
  
    Send_Data.tx[2 + i*3] = pos_tmp;       // 位置8位
    Send_Data.tx[3 + i*3] = vel_tmp;       // 速度8位
    Send_Data.tx[4 + i*3] = tor_tmp;       // 力矩8位
  }
    Send_Data.tx[38]=Check_Sum(38,SEND_DATA_CHECK);  
    tx_buffer.insert(tx_buffer.end(), 
                          Send_Data.tx, 
                          Send_Data.tx + sizeof(Send_Data.tx));
    try {
      if(!tx_buffer.empty()) {
            // 添加数据打印
            std::stringstream ss;
            ss << "发送数据[" << tx_buffer.size() << "字节]: ";
            for (auto &b : tx_buffer) {
                ss << std::hex << std::setw(2) << std::setfill('0') 
                   << static_cast<int>(b) << " ";
            }
            state->serial_tip = ss.str();

            serial_motor_->port()->send(tx_buffer);
        }
    }
        catch (const std::exception &e) {
          state->serial_tip = "发送异常: " + std::string(e.what());
        }
}

void robot::send_motor_data_xm(const LowlevelCmd_XM *cmd, LowlevelState_XM *state)
{   

  std::vector<uint8_t> tx_buffer;
  tx_buffer.reserve(51);

  Send_Data_xm.tx[0] = (FRAME_HEADER);
  Send_Data_xm.tx[1] = (0xA5); 
  for(int i=0; i<16; i++) 
  {            
    auto& motor = motors[i];
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

    pos_tmp = float_to_uint(motor.pos_set, P_MIN2, P_MAX2, 8); 
    vel_tmp = float_to_uint(motor.vel_set, V_MIN2, V_MAX2, 8); 
    tor_tmp = float_to_uint(motor.tor_set, T_MIN2, T_MAX2, 8); 
  
    Send_Data_xm.tx[2 + i*3] = pos_tmp;       // 位置8位
    Send_Data_xm.tx[3 + i*3] = vel_tmp;       // 速度8位
    Send_Data_xm.tx[4 + i*3] = tor_tmp;       // 力矩8位
  }
    Send_Data_xm.tx[50]=Check_Sum(50,SEND_DATA_CHECK);  
    tx_buffer.insert(tx_buffer.end(), 
                          Send_Data_xm.tx, 
                          Send_Data_xm.tx + sizeof(Send_Data_xm.tx));
    try {
      if(!tx_buffer.empty()) {
            // 添加数据打印
            std::stringstream ss;
            ss << "发送数据[" << tx_buffer.size() << "字节]: ";
            for (auto &b : tx_buffer) {
                ss << std::hex << std::setw(2) << std::setfill('0') 
                   << static_cast<int>(b) << " ";
            }
            state->serial_tip = ss.str();

            serial_motor_->port()->send(tx_buffer);
        }
    }
        catch (const std::exception &e) {
          state->serial_tip = "发送异常: " + std::string(e.what());
        }
}

void robot::send_set_data(const LowlevelCmd *cmd, LowlevelState *state)
{   

  // 预分配缓冲区（假设12个电机，每个电机11字节）
  std::vector<uint8_t> tx_buffer;
  tx_buffer.reserve(39);

  Send_Data.tx[0] = (FRAME_HEADER);
  Send_Data.tx[1] = (0xB3); 
  for(int i=0; i<12; i++) 
  {            
    auto& motor = motors[i];
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

    kp_tmp  = float_to_uint(motor.kp,   KP_MIN2, KP_MAX2, 12);
    kd_tmp  = float_to_uint(motor.kd,   KD_MIN2, KD_MAX2, 12);
  
    Send_Data.tx[2 + i*3] = kp_tmp & 0xFF;         // KP低8位
    Send_Data.tx[3 + i*3] = ((kp_tmp >> 8) & 0x0F) // KP高4位
                          | ((kd_tmp & 0x0F) << 4); // KD低4位
    Send_Data.tx[4 + i*3] = (kd_tmp >> 4) & 0xFF;  // KD高8位
  }
    Send_Data.tx[38]=Check_Sum(38,SEND_DATA_CHECK);  
    tx_buffer.insert(tx_buffer.end(), 
                          Send_Data.tx, 
                          Send_Data.tx + sizeof(Send_Data.tx));

    try {
      if(!tx_buffer.empty()) {
            serial_motor_->port()->send(tx_buffer);
        }
    }
        catch (const std::exception &e) {
        }
}

void robot::send_set_data_xm(const LowlevelCmd_XM *cmd, LowlevelState_XM *state)
{   

  // 预分配缓冲区（假设12个电机，每个电机11字节）
  std::vector<uint8_t> tx_buffer;
  tx_buffer.reserve(51);

  Send_Data_xm.tx[0] = (FRAME_HEADER);
  Send_Data_xm.tx[1] = (0xB3); 
  for(int i=0; i<16; i++) 
  {            
    auto& motor = motors[i];
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

    kp_tmp  = float_to_uint(motor.kp,   KP_MIN2, KP_MAX2, 12);
    kd_tmp  = float_to_uint(motor.kd,   KD_MIN2, KD_MAX2, 12);
  
    Send_Data_xm.tx[2 + i*3] = kp_tmp & 0xFF;         // KP低8位
    Send_Data_xm.tx[3 + i*3] = ((kp_tmp >> 8) & 0x0F) // KP高4位
                          | ((kd_tmp & 0x0F) << 4); // KD低4位
    Send_Data_xm.tx[4 + i*3] = (kd_tmp >> 4) & 0xFF;  // KD高8位
  }
    Send_Data_xm.tx[50]=Check_Sum(50,SEND_DATA_CHECK);  
    tx_buffer.insert(tx_buffer.end(), 
                          Send_Data_xm.tx, 
                          Send_Data_xm.tx + sizeof(Send_Data_xm.tx));

    try {
      if(!tx_buffer.empty()) {
            serial_motor_->port()->send(tx_buffer);
        }
    }
        catch (const std::exception &e) {
        }
}

void robot::fresh_cmd_motor_data(double pos, double vel,double torque, double kp,double kd,int motor_idx)
{//更新发给电机的参数、力矩等
  motors[motor_idx].pos_set = pos;
  motors[motor_idx].vel_set = vel;
  motors[motor_idx].tor_set = torque;
  motors[motor_idx].kp = kp;
  motors[motor_idx].kd = kd;                
}

void robot::get_motor_data(double &pos,double &vel,double &torque, int motor_idx)
{//获取电机反馈的参数、力矩等
  pos = motors[motor_idx].pos;
  vel = motors[motor_idx].vel;
  torque =motors[motor_idx].tor;
}

/**************************************
功能: 串口通讯校验函数，数据包n有个字节，第n-1个字节为校验位，第n个字节位帧尾。第1个字节到第n-2个字节数据按位异或的结果与第n-1个字节对比，即为BCC校验
输入参数： Count_Number：数据包前几个字节加入校验   mode：对发送数据还是接收数据进行校验
***************************************/
unsigned char robot::Check_Sum(unsigned char Count_Number,unsigned char mode)
{
    unsigned char check_sum=0,k;

    if(mode==0) //Receive data mode //接收数据模式
    {
        for(k=0;k<Count_Number;k++)
        {
            check_sum=check_sum^Receive_Data.rx[k]; //By bit or by bit //按位异或
        }
    }

    if(mode==1) //Send data mode //发送数据模式
    {
        for(k=0;k<Count_Number;k++)
        {
            check_sum=check_sum^Send_Data.tx[k]; //By bit or by bit //按位异或
        }
    }
  return check_sum; //Returns the bitwise XOR result //返回按位异或结果
}


void robot::dm4310_fbdata(motor_data_t& moto,uint8_t *data)
{
  moto.p_int=(data[0]<<8)|data[1];
  moto.v_int=(data[2]<<4)|(data[3]>>4);
  moto.t_int=((data[3]&0x0F)<<8)|data[4];
  moto.pos = uint_to_float(moto.p_int, P_MIN1, P_MAX1, 16); // (-12.5,12.5)
  moto.vel = uint_to_float(moto.v_int, V_MIN1, V_MAX1, 12); // (-30.0,30.0)
  moto.tor = uint_to_float(moto.t_int, T_MIN1, T_MAX1, 12);  // (-10.0,10.0)
}

void robot::dm4340_fbdata(motor_data_t& moto,uint8_t *data)
{
  moto.p_int=(data[0]<<8)|data[1];
  moto.v_int=(data[2]<<4)|(data[3]>>4);
  moto.t_int=((data[3]&0x0F)<<8)|data[4];
  moto.pos = uint_to_float(moto.p_int, P_MIN2, P_MAX2, 16); // (-12.5,12.5)
  moto.vel = uint_to_float(moto.v_int, V_MIN2, V_MAX2, 12); // (-30.0,30.0)
  moto.tor = uint_to_float(moto.t_int, T_MIN2, T_MAX2, 12);  // (-10.0,10.0)
}

void robot::dm6006_fbdata(motor_data_t& moto,uint8_t *data)
{
  moto.p_int=(data[0]<<8)|data[1];
  moto.v_int=(data[2]<<4)|(data[3]>>4);
  moto.t_int=((data[3]&0x0F)<<8)|data[4];
  moto.pos = uint_to_float(moto.p_int, P_MIN3, P_MAX3, 16); // (-12.5,12.5)
  moto.vel = uint_to_float(moto.v_int, V_MIN3, V_MAX3, 12); // (-30.0,30.0)
  moto.tor = uint_to_float(moto.t_int, T_MIN3, T_MAX3, 12);  // (-10.0,10.0)
}

void robot::dm8006_fbdata(motor_data_t& moto,uint8_t *data)
{
  moto.p_int=(data[0]<<8)|data[1];
  moto.v_int=(data[2]<<4)|(data[3]>>4);
  moto.t_int=((data[3]&0x0F)<<8)|data[4];
  moto.pos = uint_to_float(moto.p_int, P_MIN4, P_MAX4, 16); // (-12.5,12.5)
  moto.vel = uint_to_float(moto.v_int, V_MIN4, V_MAX4, 12); // (-30.0,30.0)
  moto.tor = uint_to_float(moto.t_int, T_MIN4, T_MAX4, 12);  // (-10.0,10.0)
}

int16_t robot::float_to_uint(float x_float, float x_min, float x_max, int bits)
{
  /* Converts a float to an unsigned int, given range and number of bits */
  float span = x_max - x_min;
  float offset = x_min;
  return (int16_t)((x_float-offset)*((float)((1<<bits)-1))/span);
}

float robot::uint_to_float(int x_int, float x_min, float x_max, int bits)
{
  /* converts unsigned int to float, given range and number of bits */
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

void robot::sendRecv(const LowlevelCmd *cmd, LowlevelState *state, int id) {
    std::lock_guard<std::mutex> lock(motor_mutex_);
    // 将通用指令转换为具体电机协议
    for(int i=0; i<12; i++){
        fresh_cmd_motor_data(
            cmd->motorCmd[i].q, 
            cmd->motorCmd[i].dq,
            cmd->motorCmd[i].tau,
            cmd->motorCmd[i].Kp,
            cmd->motorCmd[i].Kd,
            i
        );
    }
    if(id == 1)
    send_motor_data(cmd, state);  // 发送指令
    else if(id == 2)
    send_set_data(cmd, state);  // 发送指令

    
    // 接收反馈数据
   // get_motor_data_thread(cmd, state); 
    
    // // 更新状态数据
    // for(int i=0; i<12; i++){
    //     state->motorState[i].q = motors[i].pos;
    //     state->motorState[i].dq = motors[i].vel;
    //     state->motorState[i].tauEst = motors[i].tor;
    // }
}


void robot::sendRecv_xm(const LowlevelCmd_XM *cmd, LowlevelState_XM *state, int id) {
    std::lock_guard<std::mutex> lock(motor_mutex_);

    // 将通用指令转换为具体电机协议
    for(int i=0; i<16; i++){
        fresh_cmd_motor_data(
            cmd->motorCmd[i].q, 
            cmd->motorCmd[i].dq,
            cmd->motorCmd[i].tau,
            cmd->motorCmd[i].Kp,
            cmd->motorCmd[i].Kd,
            i
        );
    }

    if(id == 1)
    send_motor_data_xm(cmd, state);  // 发送指令
    else if(id == 2)
    send_set_data_xm(cmd, state);  // 发送指令

}
