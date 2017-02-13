#pragma once
#include <stdint.h>

#define 	LM_MAX_UUDP_PAYLOAD			270
#define 	LM_MAX_UUDP_DATA_QUEUE		(LM_MAX_UUDP_PAYLOAD * 5)

extern int32_t 	g_UudpRecvPacketLoss;	

typedef struct YJ_UudpCore YJ_UudpCore;

/* =========================================================
		初始化Uudp
		return
			句柄		成功
			NULL	失败

			int32_t BaudRate 波特率
===========================================================*/
YJ_UudpCore * YJ_UudpInit(int32_t BaudRate);

/* ============================================
		反初始化Uudp
=============================================*/
void YJ_UudpUninit(YJ_UudpCore * h);

/* ============================================
		接收Uudp数据包
		return
			数据长度		成功
			0			没有
			-1			缓存长度太短

		param
		char 	*buf	数据缓存
		int32_t buflen	缓存长度
=============================================*/
int32_t YJ_UudpRecv(YJ_UudpCore * h, char *buf, int32_t buflen);

/* ============================================
		向Uudp发送数据
		return
			1		成功
			0		失败

		param
		char 	*data	发送数据
		int32_t datalen	数据长度
=============================================*/
int32_t YJ_UudpSend(YJ_UudpCore * h, char *data, int32_t datalen);

