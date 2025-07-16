
	
#include "connect_task.h"

#include "cmsis_os.h"
#include "bsp_usart1.h"
#include "usbd_cdc_if.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
extern UART_HandleTypeDef huart1;
extern send_data_t send_data;													 																														 															 														 

uint32_t OBSERVE_TIME=1;
															 
void Connect_task(void)
{
  while(1)
	{  	
		send_data.tx[0]=FRAME_HEADER;
		for(int i=0;i<14;i++)
		{
			send_data.tx[1+i*5]=motor[i].para.p_int>>8;
			send_data.tx[2+i*5]=motor[i].para.p_int;
			send_data.tx[3+i*5]=motor[i].para.v_int>>4;
			send_data.tx[4+i*5]=((motor[i].para.v_int&0x0F)<<4)|(motor[i].para.t_int>>8);
			send_data.tx[5+i*5]=motor[i].para.t_int;
		}
		
		send_data.tx[51]=Check_Sum(51,send_data.tx); 
		
		//HAL_UART_Transmit_DMA(&huart1, (uint8_t *)send_data.tx, sizeof(send_data.tx));
		
		CDC_Transmit_HS((uint8_t *)send_data.tx,sizeof(send_data.tx));
		
	  osDelay(OBSERVE_TIME);
	}
}


