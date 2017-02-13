#pragma once
#include <stdio.h>
#include <stdlib.h>
#include "stm32f10x.h"

/* =============================================
		设置超时时间
================================================*/
void YJ_SetTimeOut_ms(uint64_t * handle, uint64_t ms);

/* =============================================
		判断是否超时
			1		超时
			0
================================================*/
int32_t YJ_IsTimeOut(uint64_t handle);

