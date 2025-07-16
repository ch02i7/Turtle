#include "chassisLB_task.h"
#include "fdcan.h"
#include "cmsis_os.h"

void ChassisLB_task(void)
{
	osDelay(2000);
	for(int j=0;j<10;j++)
	{

		dm_motor_enable(&hfdcan2, &motor[10]);
		osDelay(20);
	}
	for(int j=0;j<10;j++)
	{

		dm_motor_enable(&hfdcan2, &motor[11]);
		osDelay(20);
	}
	for(int j=0;j<10;j++)
	{

		dm_motor_enable(&hfdcan2, &motor[12]);
		osDelay(20);
	}
	for(int j=0;j<10;j++)
	{

		dm_motor_enable(&hfdcan2, &motor[13]);
		osDelay(20);
	}
	
	while(1)
	{	
		motor[11].ctrl.pos_set=0.1f;
		dm_motor_ctrl_send(&hfdcan2, &motor[11]);
		osDelay(1000); 
		motor[11].ctrl.pos_set=0.1f;
		dm_motor_ctrl_send(&hfdcan2, &motor[11]);
		osDelay(1000); 
		motor[11].ctrl.pos_set=0.1f;
		dm_motor_ctrl_send(&hfdcan2, &motor[11]);
		osDelay(1000); 

		dm_motor_ctrl_send(&hfdcan2, &motor[10]);
		dm_motor_ctrl_send(&hfdcan2, &motor[11]);
		dm_motor_ctrl_send(&hfdcan2, &motor[12]);
		dm_motor_ctrl_send(&hfdcan2, &motor[13]);

		osDelay(10); 
	}

}

