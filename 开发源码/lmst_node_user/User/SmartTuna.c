/*
 * SmartTuna.c
 *
 * Created: 2016/9/2 
 *  Author: YJ
 * Email: huangyj@lzrobot.com
 */ 
 
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include "stm32f10x.h"
#include "core_cm3.h"

#include "cJSON.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "SmartTuna.h"
#include "LMST_MotionCtrl.h"
#include "LM_Cudp_m3.h"
#include "LM_Uudp.h"
#include "LM_SystemClock.h"
#include "LM_Common.h"
#include "LMST_HeadNode.h"
#include "LMST_NodeCommon.h"
#include "LMST_HeadNode.h"
#include "LMST_TailNode.h"
#include "LMST_ServoMotorNode.h"
#include "LMST_SensorNode.h"
#include "LMST_HeadNode_LocalCtrl.h"
#include "LMST_DivingNode.h"

//const int32_t LMST_NODE_TYPE  =		LM_LOCAL_CAN_ADDR;	

static void delay_ms(u16 time)
{    
   u16 i=0;  
   while(time--)
   {
      i=12000;  //自己定义
      while(i--) ;    
   }
}

int main(void)
{
	void (* MainTask)(void *);
	
	delay_ms(400);
	
	YJ_InitSystem();
	
	#if _LMST_NODE_TYPE == LMST_HEAD_NODE
	MainTask = YJ_HeadNodeTask;
	#elif _LMST_NODE_TYPE == LMST_HEAD_LOCAL_CTRL_NODE
	MainTask = YJ_HeadNodeTask_LocalCtrl;
	#elif _LMST_NODE_TYPE == LMST_TAIL_NODE
	MainTask = YJ_TailNodeTask;
	#elif _LMST_NODE_TYPE == LMST_SERVO_MOTOR_NODE
	MainTask = YJ_ServoMotorNodeTask;
	#elif _LMST_NODE_TYPE == LMST_SENSOR_NODE
	MainTask = YJ_SensorNodeTask;
	#elif _LMST_NODE_TYPE == LMST_DIVING_NODE
	MainTask = YJ_DivingNodeTask;	
	#endif

	xTaskCreate( MainTask, ( signed portCHAR * ) "MainTask", 
		1024, NULL, 
			LMST_NODE_TASK_PRIORITY, NULL);
		
	vTaskStartScheduler();
	return 0;
}


