/*
 * MST_MotionCtrl.c
 *
 * Created: 2016/9/2 
 *  Author: YJ
 * Email: huangyj@lzrobot.com
 */ 
 
#include "LMST_MotionCtrl.h"
#include "LMST_Pwm.h"
#include "OnChip.h"

#include <stdio.h>
#include <stdlib.h>
#include "stm32f10x.h"
#include "SmartTuna.h"
#include "stm32f10x.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

struct YJ_MtctCore
{
	YJ_MtctParam 	param;
	int32_t 			IsInited;
	int32_t 			IsExited;
	TaskHandle_t 	MtctTaskHandle;
};

static YJ_MtctCore g_handle = {0};

/*=================================================
        cpg算法
==================================================*/
static int32_t DoCpgCaculate(YJ_MtctParam p, double S[JointCount])
{
		u8 i;
		CalculateSitaLevel(1,10,10,p.Speed,p.Direct);
		
	  if(p.Speed==0)
		{
			ResetData();
			
			for(i=0;i<JointCount;i++)
			{
				Sita[i]=KSita*XTem[i][p.Direct];
			}
		}

		for(i=0;i<JointCount;i++)
		{
			S[i] = Sita[i];
		}
			
		return 1;
}

/*=================================================
        TailNode 运动控制线程
==================================================*/
static void YJ_MtctTask_TailNode( void *parameters )
{
	double 				S[JointCount];
  const portTickType xDelay = pdMS_TO_TICKS(20);  //宏pdMS_TO_TICKS用于将毫秒转成节拍数
	
	YJ_MtctCore  * h = (YJ_MtctCore *)parameters;
	
//	h->param.Speed = 15;
	//周期性执行cpg算法
	while(!h->IsExited)
	{
		DoCpgCaculate(h->param, S);
		YJ_PwmSet_tail(S[2] + h->param.ServoOffset[2]);
		vTaskDelay( xDelay );
		//YJ_UnInitSystem();
		//YJ_ReBoot();
	}
	vTaskDelete(NULL);
}

/*=================================================
        ServoMotor运动控制线程
==================================================*/
static void YJ_MtctTask_ServoMotorNode( void *parameters )
{
	double 				S[JointCount];
  const portTickType xDelay = pdMS_TO_TICKS(20);  //宏pdMS_TO_TICKS用于将毫秒转成节拍数
	
	YJ_MtctCore  * h = (YJ_MtctCore *)parameters;
	
//	h->param.Speed = 15;
	h->param.LMotor = 7;
	h->param.RMotor = 7;
	while(!h->IsExited)
	{
		YJ_PwmSet_sm(h->param.LServo, h->param.RServo, h->param.LMotor, h->param.RMotor);
		vTaskDelay( xDelay );
	}
	vTaskDelete(NULL);
}

/*=================================================
        运动控制初始化
    return
        句柄           成功
        NULL           失败
==================================================*/
YJ_MtctCore * YJ_MtctInit()
{
	YJ_MtctCore * h;
	void (* Task)(void *);
	
	h = &g_handle;//calloc(1,sizeof(YJ_MtctCore));
		
	memset(h, 0, sizeof(YJ_MtctCore));
	if(h == NULL)
		return NULL;
	
	ReadParameter(h->param.ServoOffset);
	
	h->param.Direct = 7;
	h->param.LServo = 7;
	h->param.RServo = 7;
	h->param.LMotor = 7;
	h->param.RMotor = 7;
	
	YJ_PwmInit();
	
	#if _LMST_NODE_TYPE == LMST_TAIL_NODE
	Task = YJ_MtctTask_TailNode;
	#elif _LMST_NODE_TYPE == LMST_SERVO_MOTOR_NODE
	Task = YJ_MtctTask_ServoMotorNode;
	#endif
	
	xTaskCreate( Task, ( signed portCHAR * ) "YJ_MtctTask", 
		configMINIMAL_STACK_SIZE, (void *)h, 
			LMST_MTCT_TASK_PRIORITY, &h->MtctTaskHandle);
		
	h->IsInited = 1;
	return h;
}

/*=================================================
        运动控制反初始化
==================================================*/
void YJ_MtctUnInit(YJ_MtctCore *h)
{
	if(!h->IsInited)
		return;
	
	//vTaskDelete(h->MtctTaskHandle);
	YJ_PwmUnInit();
	h->IsExited = 1;
	
}

/*=================================================
        设置控制参数
    return
        1           成功
        0           失败
==================================================*/
int32_t YJ_MtctSetParam(YJ_MtctCore *h, YJ_MtctParam * p)
{
	memcpy(&h->param, p, sizeof(YJ_MtctParam));
	return 1;
}

/*=================================================
        获取控制参数
    return
        1           成功
        0           失败
==================================================*/
int32_t YJ_MtctGetParam(YJ_MtctCore *h, YJ_MtctParam * p)
{
	memcpy( p, &h->param, sizeof(YJ_MtctParam));
	return 1;
}

/*=================================================
        把尾鳍的偏移量保存到flash里
==================================================*/
void YJ_SaveTraiFinOffset(YJ_MtctCore *h)
{
	SaveParameter(h->param.ServoOffset);
}

