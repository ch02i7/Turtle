/**
  *********************************************************************
  * @file      chassisL_task.c/h
  * @brief     该任务控制左腿的五个电机，都是DM4340，这五个电机挂载在can2总线上
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
#include "chassisL_task.h"
#include "fdcan.h"
#include "cmsis_os.h"


void ChassisL_task(void)
{

	osDelay(2000);
	for(int j=0;j<10;j++)
	{

		dm_motor_enable(&hfdcan2, &motor[7]);
		osDelay(20);
	}
	for(int j=0;j<10;j++)
	{

		dm_motor_enable(&hfdcan2, &motor[8]);
		osDelay(20);
	}
	for(int j=0;j<10;j++)
	{

		dm_motor_enable(&hfdcan2, &motor[9]);
		osDelay(20);
	}
	
	while(1)
	{	motor[2].ctrl.tor_set = 1.0f;
		dm_motor_ctrl_send(&hfdcan2, &motor[7]);
		dm_motor_ctrl_send(&hfdcan2, &motor[8]);
		dm_motor_ctrl_send(&hfdcan2, &motor[9]);
		osDelay(1); 
	}
}
