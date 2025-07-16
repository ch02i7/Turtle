#include "chassisR_task.h"
#include "fdcan.h"
#include "cmsis_os.h"
#include "dm_motor_ctrl.h"

void ChassisR_task(void)
{
	osDelay(2000);
	for(int j=0;j<10;j++)
	{

		dm_motor_enable(&hfdcan1, &motor[0]);
		osDelay(20);
	}
	for(int j=0;j<10;j++)
	{
	dm_motor_enable(&hfdcan1, &motor[1]);
		osDelay(20);
	}
	for(int j=0;j<10;j++)
	{

		dm_motor_enable(&hfdcan1, &motor[2]);
		osDelay(20);
	}

	while(1)
	{	
		dm_motor_ctrl_send(&hfdcan1, &motor[0]);
		dm_motor_ctrl_send(&hfdcan1, &motor[1]);
		dm_motor_ctrl_send(&hfdcan1, &motor[2]);
		osDelay(1); 
	}
}

