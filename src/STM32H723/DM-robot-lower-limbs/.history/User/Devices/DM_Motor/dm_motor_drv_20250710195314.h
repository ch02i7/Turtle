#ifndef __DM_MOTOR_DRV_H__
#define __DM_MOTOR_DRV_H__
#include "main.h"
#include "fdcan.h"
#include "bsp_fdcan.h"

#define MIT_MODE 			0x000
#define POS_MODE			0x100
#define SPD_MODE			0x200
#define PSI_MODE		  	0x300

#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

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


typedef enum
{
    Motor1,
    Motor2,
    Motor3,
    Motor4,
    Motor5,
    Motor6,
	Motor7,
	Motor8,
	Motor9,
	Motor10,
    Motor11,
    Motor12,
    Motor13,
	Motor14,
    num
} motor_num;

typedef enum
{
	mit_mode = 1,
	pos_mode = 2,
	spd_mode = 3,
	psi_mode = 4
} mode_e;

typedef enum {
    RID_UV_VALUE=0,    // ��ѹ����ֵ
    RID_KT_VALUE=1,    // Ť��ϵ��
    RID_OT_VALUE=2,    // ���±���ֵ
    RID_OC_VALUE=3,    // ��������ֵ
    RID_ACC		=4,    // ���ٶ�
    RID_DEC		=5,    // ���ٶ�
    RID_MAX_SPD	=6,    // ����ٶ�
    RID_MST_ID	=7,    // ����ID
    RID_ESC_ID	=8,    // ����ID
    RID_TIMEOUT	=9,    // ��ʱ����ʱ��
    RID_CMODE	=10,   // ����ģʽ
    RID_DAMP	=11,   // ���ճ��ϵ��
    RID_INERTIA =12,   // ���ת������
    RID_HW_VER	=13,   // ����
    RID_SW_VER	=14,   // �����汾��
    RID_SN		=15,   // ����
    RID_NPP		=16,   // ���������
    RID_RS		=17,   // ����
    RID_LS		=18,   // ���
    RID_FLUX	=19,   // ����
    RID_GR		=20,   // ���ּ��ٱ�
    RID_PMAX	=21,   // λ��ӳ�䷶Χ
    RID_VMAX	=22,   // �ٶ�ӳ�䷶Χ
    RID_TMAX	=23,   // Ť��ӳ�䷶Χ
    RID_I_BW	=24,   // ���������ƴ���
    RID_KP_ASR	=25,   // �ٶȻ�Kp
    RID_KI_ASR	=26,   // �ٶȻ�Ki
    RID_KP_APR	=27,   // λ�û�Kp
    RID_KI_APR	=28,   // λ�û�Ki
    RID_OV_VALUE=29,   // ��ѹ����ֵ
    RID_GREF	=30,   // ��������Ч��
    RID_DETA	=31,   // �ٶȻ�����ϵ��
    RID_V_BW	=32,   // �ٶȻ��˲�����
    RID_IQ_CL	=33,   // ��������ǿϵ��
    RID_VL_CL	=34,   // �ٶȻ���ǿϵ��
    RID_CAN_BR	=35,   // CAN�����ʴ���
    RID_SUB_VER	=36,   // �Ӱ汾��
    RID_U_OFF	=50,   // u��ƫ��
    RID_V_OFF	=51,   // v��ƫ��
    RID_K1		=52,   // ��������1
    RID_K2		=53,   // ��������2
    RID_M_OFF	=54,   // �Ƕ�ƫ��
    RID_DIR		=55,   // ����
    RID_P_M		=80,   // ���λ��
    RID_X_OUT	=81    // �����λ��
} rid_e;

// 电调参数配置结构体（用于读写电机内部参数）
typedef struct
{
    uint8_t read_flag;   // 参数读取成功标志 (1:成功)
    uint8_t write_flag;  // 参数写入成功标志 (1:成功)
    uint8_t save_flag;   // 参数保存成功标志 (1:成功)
    
    // 基础保护参数
    float UV_Value;      // 欠压保护阈值（单位：V）
    float KT_Value;      // 扭矩系数（单位：N·m/A）
    float OT_Value;      // 过温保护阈值（单位：℃）
    float OC_Value;      // 过流保护阈值（单位：A）
    
    // 运动控制参数
    float ACC;          // 加速度（单位：rad/s²）
    float DEC;          // 减速度（单位：rad/s²）
    float MAX_SPD;      // 最大速度限制（单位：rad/s）
    
    // 设备标识参数
    uint32_t MST_ID;    // 主控ID（CAN通信ID，范围：0x001-0x7FF）
    uint32_t ESC_ID;    // 电调ID（CAN通信ID，范围：0x001-0x7FF）
    uint32_t TIMEOUT;   // 通信超时时间（单位：ms）
    
    // 控制模式参数
    uint32_t cmode;     // 控制模式标识（1:MIT 2:位置 3:速度 4:PSI）
    float Damp;         // 阻尼系数（无量纲，范围：0.0-1.0）
    float Inertia;      // 转动惯量（单位：kg·m²）
    
    // 版本信息
    uint32_t hw_ver;    // 硬件版本号（格式：V1.2.3）
    uint32_t sw_ver;    // 固件版本号（格式：V1.2.3）
    uint32_t SN;        // 产品序列号
    
    // 电机特性参数
    uint32_t NPP;       // 电机极对数（永磁同步电机参数）
    float Rs;           // 定子电阻（单位：Ω）
    float Ls;           // 定子电感（单位：H）
    float Flux;         // 永磁体磁链（单位：Wb）
    float Gr;           // 减速箱传动比
    
    // 量程范围参数
    float PMAX;         // 位置量程范围（单位：rad）
    float VMAX;         // 速度量程范围（单位：rad/s）
    float TMAX;         // 扭矩量程范围（单位：N·m）
    
    // 控制环参数
    float I_BW;         // 电流环带宽（单位：Hz）
    float KP_ASR;       // 速度环比例系数
    float KI_ASR;       // 速度环积分系数
    float KP_APR;       // 位置环比例系数
    float KI_APR;       // 位置环积分系数
    
    // 高级保护参数
    float OV_Value;     // 过压保护阈值（单位：V）
    float GREF;         // 重力补偿系数（单位：N·m）
    
    // 滤波器参数
    float Deta;         // 速度环死区系数
    float V_BW;         // 速度环滤波器带宽（单位：Hz）
    
    // 电流限制参数
    float IQ_cl;        // q轴电流限幅系数（范围：0.0-1.0）
    float VL_cl;        // 速度限幅系数（范围：0.0-1.0）
    
    // CAN配置
    uint32_t can_br;    // CAN波特率代码（0:1Mbps 1:500kbps 2:250kbps...）
    uint32_t sub_ver;   // 子版本号
    
    // 电机校准参数
    float u_off;        // U相电压偏置（单位：V）
    float v_off;        // V相电压偏置（单位：V）
    float k1;           // 非线性补偿系数1
    float k2;           // 非线性补偿系数2
    float m_off;        // 机械角度偏置（单位：rad）
    float dir;          // 旋转方向（1:正方向，-1:反方向）
    
    // 位置反馈参数
    float p_m;          // 机械角度（单位：rad）
    float x_out;        // 输出轴角度（单位：rad）
} esc_inf_t;


// 电机反馈参数结构体（实时数据）
typedef struct
{
    int id;          // 电机ID (0-15)
    int state;       // 状态码 (0:正常, 1:过温, 2:过流, 4:超速)
    int p_int;       // 原始位置整数值 (16bit 原始数据)
    int v_int;       // 原始速度整数值 (12bit 原始数据)
    int t_int;       // 原始扭矩整数值 (12bit 原始数据)
    int kp_int;      // Kp系数整数值 (12bit 原始数据)
    int kd_int;      // Kd系数整数值 (12bit 原始数据)
    float pos;       // 换算后实际位置（单位：弧度，范围：-4π~4π）
    float vel;       // 换算后实际速度（单位：rad/s，范围：-30~30）
    float tor;       // 换算后实际扭矩（单位：N·m，范围：-12~12）
    float Kp;        // 比例系数（范围：KP_MIN2~KP_MAX2）
    float Kd;        // 微分系数（范围：KD_MIN2~KD_MAX2）
    float Tmos;      // 电机温度（单位：℃）
    float Tcoil;     // 线圈温度（单位：℃）
} motor_fbpara_t;

// 电机控制参数结构体（设定值）
typedef struct
{
    uint8_t mode;    // 控制模式 (1:MIT 2:位置 3:速度 4:PSI)
    float pos_set;   // 目标位置（单位：弧度）
    float vel_set;   // 目标速度（单位：rad/s）
    float tor_set;   // 目标扭矩（单位：N·m）
    float cur_set;   // 预留电流设定值
    float kp_set;    // 比例系数设定值
    float kd_set;    // 微分系数设定值
} motor_ctrl_t;

// 电机完整描述结构体
typedef struct
{
    uint16_t id;         // CAN总线ID (0x001~0x1FF)
    uint16_t mst_id;     // 主控ID (默认0x001)
    motor_fbpara_t para; // 实时反馈参数
    motor_ctrl_t ctrl;   // 控制参数
    esc_inf_t tmp;       // 临时存储区（用于参数配置过程）
} motor_t;

float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x_float, float x_min, float x_max, int bits);
void dm_motor_ctrl_send(hcan_t* hcan, motor_t *motor);
void dm_motor_enable(hcan_t* hcan, motor_t *motor);
void dm_motor_disable(hcan_t* hcan, motor_t *motor);
void dm_motor_clear_para(motor_t *motor);
void dm_motor_clear_err(hcan_t* hcan, motor_t *motor);
void dm_motor_fbdata(motor_t *motor, uint8_t *rx_data);

void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);

void mit_ctrl(hcan_t* hcan, motor_t *motor, uint16_t motor_id, float pos, float vel,float kp, float kd, float tor);
void pos_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel);
void spd_ctrl(hcan_t* hcan, uint16_t motor_id, float vel);
void psi_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel, float cur);
	
void save_pos_zero(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
void clear_err(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);

void read_motor_data(uint16_t id, uint8_t rid);
void read_motor_ctrl_fbdata(uint16_t id);
void write_motor_data(uint16_t id, uint8_t rid, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);
void save_motor_data(uint16_t id, uint8_t rid);

void dm4340_fbdata_test(motor_t *motor, uint8_t *rx_data, int i);
uint8_t set_motor_zero_offset(uint8_t pos_tmp, int i);

#endif /* __DM_MOTOR_DRV_H__ */

