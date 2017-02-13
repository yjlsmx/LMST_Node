/*
 * LM_BuoyantDevices.c
 *
 * Created: 2016/9/2 
 *  Author: YJ
 * Email: huangyj@lzrobot.com
 */
 
#include "cJSON.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "SmartTuna.h"
#include "LMST_TailNode.h"
#include "LMST_NodeCommon.h"
#include "LM_BatteryManager.h"	
#include "LM_Common.h"
#include "LM_BuoyantDevices.h"

struct YJ_ButdCore
{
	int32_t IsInited;
	int32_t	state;  	// 0 保持 1 充气 2 放气
};

// 放气
static void deflate()
{
    GPIO_WriteBit(GPIOA,GPIO_Pin_0,0);
    GPIO_WriteBit(GPIOA,GPIO_Pin_8,1);
}
// 充气
static void inflate()
{
    GPIO_WriteBit(GPIOA,GPIO_Pin_0,1);
    GPIO_WriteBit(GPIOA,GPIO_Pin_8,0);
}
// 保持
static void hold()
{
    GPIO_WriteBit(GPIOA,GPIO_Pin_0,1);
    GPIO_WriteBit(GPIOA,GPIO_Pin_8,1);
}

/* =========================================================
		Uudp接收线程
===========================================================*/
static void ButdTask( void *parameters )
{
	const portTickType xDelay = pdMS_TO_TICKS(5); 
	YJ_ButdCore *h = 	(YJ_ButdCore *)parameters;
	
	while(1)
	{
		vTaskDelay( xDelay );
		switch(h->state)
		{
			case 0:
				hold();
				break;
			
			case 1:
				inflate();
				break;

			case 2:
				deflate();
				break;

			default:
				
				break;			
		}
	}
	
	vTaskDelete(NULL);
}

static void InitGpio(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_0;	
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;					
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/* =========================================================
		初始化浮力装置
		return
			句柄		成功
			NULL	失败
===========================================================*/
YJ_ButdCore * YJ_ButdInit()
{
	YJ_ButdCore *h = (YJ_ButdCore *)pvPortMalloc(sizeof(struct YJ_ButdCore));
	
	if(h == NULL)
		return NULL;
	
	memset(h, 0, sizeof(struct YJ_ButdCore));
	InitGpio();
	
	xTaskCreate( ButdTask, ( signed portCHAR * ) "ButdTask", 
	configMINIMAL_STACK_SIZE, (void*)h, 
		LMST_BUTD_TASK_PRIORITY, NULL);
	
	h->IsInited = 1;
	
	return h;
}

/* =========================================================
		获取浮力装置状态
		return
			状态
===========================================================*/
int32_t GetButdState(YJ_ButdCore * h)
{
	if(h == NULL || !h->IsInited)
		return 0;
	
	return h->state;
}

/* =========================================================
		设置浮力装置状态
		return
			1		成功
			0   失败
===========================================================*/
int32_t SetButdState(YJ_ButdCore * h, int32_t s)
{
	if(h == NULL || !h->IsInited)
		return 0;
	
	h->state = s;
	
	return 1; 
}

/* =========================================================
		反初始化浮力装置
===========================================================*/
void YJ_ButdUnInit(YJ_ButdCore * h)
{
	return;
}




