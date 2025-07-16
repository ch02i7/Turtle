#ifndef VISION_UART_H
#define VISION_UART_H
#include "main.h"


#define SEND_DATA_CHECK   1        
#define READ_DATA_CHECK   0         
#define FRAME_HEADER      0X7B       //Frame head //֡ͷ
#define FRAME_TAIL        0X7D       //Frame tail //֡β
#define RECEIVE_DATA_SIZE 51         
#define SEND_DATA_SIZE    52

//��λ����ROS�������ݵĽṹ��
typedef struct
{
  uint8_t tx[SEND_DATA_SIZE];  
	
}send_data_t;

//ROS����λ���������ݵĽṹ��
typedef struct      
{
  uint8_t rx[RECEIVE_DATA_SIZE];
	
}rev_data_t;


extern void connect_usart1_init(void);

extern uint8_t Check_Sum(uint8_t Count_Number,uint8_t *buffer);

#endif


