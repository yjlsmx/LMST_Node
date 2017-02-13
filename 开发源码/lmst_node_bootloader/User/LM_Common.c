/*
 * LM_Common.c
 *
 * Created: 2016/9/2 
 *  Author: YJ
 * Email: huangyj@lzrobot.com
 */
 
#include "LM_SystemClock.h"
#include "LM_Common.h"
#include "SmartTuna.h"

#include <stdio.h>
#include <stdlib.h>
#include "stm32f10x.h"

/* =============================================
		设置超时时间
================================================*/
void YJ_SetTimeOut_ms(uint64_t * handle, uint64_t ms)
{
	uint64_t now;
	
	now = YJ_GetSysClock_ms();

	*handle = now + ms;
	return;
}

/* =============================================
		判断是否超时
			1		超时
			0
================================================*/
int32_t YJ_IsTimeOut(uint64_t handle)
{
	uint64_t now;
	
	now = YJ_GetSysClock_ms();
	
	if(now > handle)
		return 1;

	return 0;

}
