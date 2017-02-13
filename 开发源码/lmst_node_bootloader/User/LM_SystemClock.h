#pragma once
#include <stdio.h>
#include <stdlib.h>
#include "stm32f10x.h"

typedef struct YJ_SysClockCore YJ_SysClockCore;

/* =========================================================
		初始化系统时钟模块
		return
			句柄		成功
			NULL	失败
===========================================================*/
YJ_SysClockCore * YJ_SysClockInit(void);

/* =========================================================
		反初始化系统时钟模块
===========================================================*/
void YJ_SysClockUninit(YJ_SysClockCore *h);

/* =========================================================
		获取从系统开始运行到现在经过的时钟长度（毫秒）
		return
			句柄		成功
			NULL	失败
===========================================================*/
uint64_t YJ_GetSysClock_ms(void);

