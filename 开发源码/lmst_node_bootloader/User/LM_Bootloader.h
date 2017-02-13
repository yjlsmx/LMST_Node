#pragma once
#include <stdint.h>

typedef struct YJ_BtldCore YJ_BtldCore;

void YJ_BtldUnInit(YJ_BtldCore * h);

/* =========================================================
		初始化bootloader模块
		return
			句柄		成功
			NULL	  失败
===========================================================*/
YJ_BtldCore * YJ_BtldInit(void);

/* =========================================================
	bootloader处理从上位机接收的信息
===========================================================*/
void YJ_BtldProMsg(YJ_BtldCore *h, char * msg, int32_t len, int32_t type);