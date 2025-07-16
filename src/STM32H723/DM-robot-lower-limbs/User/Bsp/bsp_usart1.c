#include "bsp_usart1.h"
#include "chassisR_task.h"

extern UART_HandleTypeDef huart1;

send_data_t send_data;
send_data_t send_data2;

rev_data_t rev_data;


void connect_usart1_init(void)
{
  //�����Ӿ����ڵĽ����жϣ��ú�����vision_task.c����ļ�
  //HAL_UART_Receive_IT(&huart6, uart6_rxbuf,11);
  //__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);  
	//HAL_UART_Receive_DMA(&huart1,connect_data.true_buf,RXBUFFER_LEN);  
	
	  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rev_data.rx,RECEIVE_DATA_SIZE*2);	
}

/**************************************************************************
Function: Calculates the check bits of data to be sent/received
Input   : Count_Number: The first few digits of a check; Mode: 0-Verify the received data, 1-Validate the sent data
Output  : Check result
�������ܣ�����Ҫ����/���յ�����У����
��ڲ�����Count_Number��У���ǰ��λ����Mode��0-�Խ������ݽ���У�飬1-�Է������ݽ���У��
����  ֵ��У����
**************************************************************************/
uint8_t Check_Sum(uint8_t Count_Number,uint8_t *buffer)
{
	uint8_t check_sum=0;


	for(uint8_t k=0;k<Count_Number;k++)
	{
	   check_sum=check_sum^buffer[k];
	}

	return check_sum;
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance==USART1)
	{
//		 if(Size== RECEIVE_DATA_SIZE) //Verify the length of the packet //��֤���ݰ��ĳ���
//		 {   
//			 if(rev_data.rx[0] == FRAME_HEADER) //Verify the frame tail of the packet //��֤���ݰ���֡β
//			 {
//				if(rev_data.rx[22] == FRAME_TAIL) //Verify the frame tail of the packet //��֤���ݰ���֡β
//				{
//					//Data exclusionary or bit check calculation, mode 0 is sent data check
//					//�������λУ����㣬ģʽ0�Ƿ�������У��
//					if(rev_data.rx[21] ==Check_Sum(21,rev_data.rx))	 
//				  {	
//						for(int i=0;i<10;i++)
//						{
//						  int16_t temp=0;
//						  temp=((rev_data.rx[1+2*i]<<8)|rev_data.rx[2+2*i]);
//						  chassis_move.joint_motor[i].para.tor_set=(float)((temp/1000)+(temp%1000)*0.001f);
//						}
//					}
//					else
//				  {
//				  	  memset(rev_data.rx, 0,RECEIVE_DATA_SIZE);
//				   }
//				}
//				else
//				{
//				  	memset(rev_data.rx, 0,RECEIVE_DATA_SIZE);
//				}
//			 }
//			 else
//			 {
//				  	memset(rev_data.rx, 0,RECEIVE_DATA_SIZE);
//			 }
//		 }
//		 else
//		 {
//				  memset(rev_data.rx, 0,RECEIVE_DATA_SIZE);
//			}
	  
	}
}
