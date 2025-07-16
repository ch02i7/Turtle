#include "chassisRB_task.h"
#include "fdcan.h"
#include "cmsis_os.h"

extern motor_t motor[num];

void ChassisRB_task(void)
{
	osDelay(2000);
	for(int j=0;j<10;j++)
	{

		dm_motor_enable(&hfdcan1, &motor[3]);
		osDelay(20);
	}
	for(int j=0;j<10;j++)
	{

		dm_motor_enable(&hfdcan1, &motor[4]);
		osDelay(20);
	}
	for(int j=0;j<10;j++)
	{
   
		dm_motor_enable(&hfdcan1, &motor[5]);
		osDelay(20);
	}
	for(int j=0;j<10;j++)
	{

		dm_motor_enable(&hfdcan1, &motor[6]);
		osDelay(20);
	}


	while(1)
	{	
		dm_motor_ctrl_send(&hfdcan1, &motor[3]);
		dm_motor_ctrl_send(&hfdcan1, &motor[4]);
		dm_motor_ctrl_send(&hfdcan1, &motor[5]);
		dm_motor_ctrl_send(&hfdcan1, &motor[6]);
		osDelay(10); 
	}
}
