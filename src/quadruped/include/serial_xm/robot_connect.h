#ifndef _ROBOT_H_
#define _ROBOT_H_
#include <iostream>

#include <thread>
#include <initializer_list>
#include <fstream>
#include <array>
#include <serial_driver/serial_port.hpp>
#include <serial_driver/serial_driver.hpp>
#include <sensor_msgs/msg/joint_state.hpp> 
#include "message/LowlevelCmd.h"    // 新增头文件包含
#include "message/LowlevelState.h"   // 新增头文件包含
#include "interface/IOInterface.h"


//4310
#define P_MIN1 -12.5f
#define P_MAX1 12.5f
#define V_MIN1 -30.0f
#define V_MAX1 30.0f
#define KP_MIN1 0.0f
#define KP_MAX1 500.0f
#define KD_MIN1 0.0f
#define KD_MAX1 5.0f
#define T_MIN1 -10.0f
#define T_MAX1 10.0f

//4340
#define P_MIN2  -6.25f
#define P_MAX2  6.25f
#define V_MIN2  -20.0f
#define V_MAX2  20.0f
#define KP_MIN2 0.0f
#define KP_MAX2 500.0f
#define KD_MIN2 0.0f
#define KD_MAX2 5.0f
#define T_MIN2 -10.0f
#define T_MAX2  10.0f

//6006
#define P_MIN3 -12.5f
#define P_MAX3 12.5f
#define V_MIN3 -45.0f
#define V_MAX3 45.0f
#define KP_MIN3 0.0f
#define KP_MAX3 500.0f
#define KD_MIN3 0.0f
#define KD_MAX3 5.0f
#define T_MIN3 -12.0f
#define T_MAX3 12.0f

//8006
#define P_MIN4 -12.5f
#define P_MAX4 12.5f
#define V_MIN4 -45.0f
#define V_MAX4 45.0f
#define KP_MIN4 0.0f
#define KP_MAX4 500.0f
#define KD_MIN4 0.0f
#define KD_MAX4 5.0f
#define T_MIN4 -20.0f
#define T_MAX4 20.0f 

#define SEND_DATA_CHECK   1          //Send data check flag bits //发送数据校验标志位
#define READ_DATA_CHECK   0          //Receive data to check flag bits //接收数据校验标志位
#define FRAME_HEADER      0X7B       //Frame head //帧头
#define FRAME_TAIL        0X7D       //Frame tail //帧尾

//52 27
#define RECEIVE_DATA_SIZE 72         //The length of the data sent by the lower computer //下位机发送过来的数据的长度
#define SEND_DATA_SIZE    39
#define SEND_DATA_SIZE_XM    51

//ROS向下位机发送数据的结构体
typedef struct
{
        uint8_t tx[SEND_DATA_SIZE];        
        unsigned char Frame_Tail; 
}send_data_t;

typedef struct
{
        uint8_t tx[SEND_DATA_SIZE_XM];        
        unsigned char Frame_Tail; 
}send_data_t_xm;

typedef struct      
{
        uint8_t rx[RECEIVE_DATA_SIZE];
        uint8_t Flag_Stop;
        unsigned char Frame_Header;
        unsigned char Frame_Tail;
}rev_data_t;

typedef struct      
{
    std::string name;
    std::string type;//属于哪个电机
    int index;

    float pos;  
    float vel;  
    float tor; 
    int p_int;
    int v_int;
    int t_int;
    

    float pos_set; 
    float vel_set; 
    float tor_set;
    float kp;
    float kd;
}motor_data_t;


class robot : public IOInterface { 
  
    private:
        std::string robot_name, Serial_Type;

        int motor_seial_baud;
        std::string motor_serial_port;
        std::thread rec_thread;
        rev_data_t Receive_Data;

        std::thread pub_thread;
        send_data_t Send_Data;
        send_data_t_xm Send_Data_xm;

        std::array<motor_data_t, 14> motors;

        std::shared_ptr<drivers::common::IoContext> io_ctx_;  // 使用common命名空间
        std::unique_ptr<drivers::serial_driver::SerialDriver> serial_motor_;

        std::mutex motor_mutex_;  // 添加互斥锁

    public:

        explicit robot(const std::string& port, int baud);

        void sendRecv(const LowlevelCmd *cmd, LowlevelState *state, int id) override;
        
        void sendRecv_xm(const LowlevelCmd_XM *cmd, LowlevelState_XM *state, int id) override;
        ~robot();
        
         void init_motor_serial();
        //读取串口电机数据线程  
        void get_motor_data_thread(const LowlevelCmd *cmd, LowlevelState *state);  

        //将括号里的数据放到buff里
        void fresh_cmd_motor_data(double pos, double vel,double torque, double kp,double kd,int motor_idx);
        //将buff里的数据发送给电机
        void send_motor_data(const LowlevelCmd *cmd, LowlevelState *state);
        void send_set_data(const LowlevelCmd *cmd, LowlevelState *state);
        void send_motor_data_xm(const LowlevelCmd_XM *cmd, LowlevelState_XM *state);
        void send_set_data_xm(const LowlevelCmd_XM *cmd, LowlevelState_XM *state);

        //读取buff里的数据到括号内的变量
        void get_motor_data(double &pos,double &vel,double &torque, int motor_idx);
        void publishJointStates(); 
        
        void dm4310_fbdata(motor_data_t& moto,uint8_t *data);
        void dm4340_fbdata(motor_data_t& moto,uint8_t *data);
        void dm6006_fbdata(motor_data_t& moto,uint8_t *data);
        void dm8006_fbdata(motor_data_t& moto,uint8_t *data);

        unsigned char Check_Sum(unsigned char Count_Number,unsigned char mode);      
        int16_t float_to_uint(float x_float, float x_min, float x_max, int bits);
        float uint_to_float(int x_int, float x_min, float x_max, int bits);

        std::atomic<bool> stop_thread_ ;


  };

#endif
