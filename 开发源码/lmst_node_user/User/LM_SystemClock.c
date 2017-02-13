/*
 * LM_SystemClock.c
 *
 * Created: 2016/9/2 
 *  Author: YJ
 * Email: huangyj@lzrobot.com
 */
 
#include "LM_SystemClock.h"
#include "SmartTuna.h"

#include <stdio.h>
#include <stdlib.h>
#include "stm32f10x.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

struct YJ_SysClockCore
{
	int32_t  IsExited;
	uint64_t SystemTick_ms;
};

static YJ_SysClockCore * g_SysClockCore = NULL;
/* =========================================================
		线程
===========================================================*/
static void SysClockTask( void *parameters )
{
	const portTickType xDelay = pdMS_TO_TICKS(5); 
	YJ_SysClockCore *h = 	(YJ_SysClockCore *)parameters;
	
	while(!h->IsExited)
	{
		vTaskDelay( xDelay );
		h->SystemTick_ms += 5;
	}
	
	vTaskDelete(NULL);
}

/* =========================================================
		初始化系统时钟模块
		return
			句柄		成功
			NULL	失败
===========================================================*/
YJ_SysClockCore * YJ_SysClockInit(void)
{
		YJ_SysClockCore *h = (YJ_SysClockCore *)pvPortMalloc(sizeof(struct YJ_SysClockCore));
	
	if(h == NULL)
		return NULL;
	
	memset(h, 0, sizeof(struct YJ_SysClockCore));
	
	xTaskCreate( SysClockTask, ( signed portCHAR * ) "SysClockTask", 
	  configMINIMAL_STACK_SIZE, (void*)h, 
		LMST_SYSTEM_CLOCK_TASK_PRIORITY, NULL);
		
		g_SysClockCore = h;
		
		return h;
}

/* =========================================================
		反初始化系统时钟模块
===========================================================*/
void YJ_SysClockUninit(YJ_SysClockCore *h)
{
	h->IsExited = 1;
}

/* =========================================================
		获取从系统开始运行到现在经过的时钟长度（毫秒）
		return
			句柄		成功
			NULL	失败
===========================================================*/
uint64_t YJ_GetSysClock_ms(void)
{
	if(g_SysClockCore == NULL)
		return 0;
	
	return g_SysClockCore->SystemTick_ms;
}
