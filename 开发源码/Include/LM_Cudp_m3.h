#pragma once
#include <stdint.h>

#define 	LM_MAX_CAN_PAYLOAD			256
#define 	LM_MAX_CAN_DATA_QUEUE		(LM_MAX_CAN_PAYLOAD * 5)
#define 	LM_LOCAL_CAN_ADDR				LMST_SENSOR_NODE//LMST_HEAD_NODE//LMST_SERVO_MOTOR_NODE//LMST_DIVING_NODE//LMST_TAIL_NODE//LMST_HEAD_LOCAL_CTRL_NODE//

typedef struct YJ_CudpCore YJ_CudpCore;

/* =========================================================
		初始化一个cudp
		return
			句柄		成功
			NULL		失败
===========================================================*/
YJ_CudpCore * YJ_CudpInit(void);

/* =========================================================
		反初始化cudp
===========================================================*/
void YJ_CudpUnInit(YJ_CudpCore * h);

/* ============================================
		接收cudp数据
		return
			数据长度		成功
			0			没有
			-1			缓存长度太短

		param
		char 	*buf	数据缓存
		int32_t buflen	缓存长度
=============================================*/
int32_t YJ_CudpRecv(YJ_CudpCore * h, char *buf, int32_t buflen);

/* ============================================
		向cudp发送数据
		return
			1		成功
			0		失败

		param
		char 	*data	发送数据
		int32_t datalen	数据长度
=============================================*/
int32_t YJ_CudpSend(YJ_CudpCore * h, char *data, int32_t datalen);

/* ============================================
		接收cudp数据
		return
			数据长度		成功
			0			没有
			-1			缓存长度太短

		param
		char 	*buf	    数据缓存
		int32_t buflen	缓存长度
		int32_t *addr_src 源地址  
=============================================*/
int32_t YJ_CudpRecvFrom(YJ_CudpCore * h, char *buf, int32_t buflen, int32_t *addr_src);

/* ============================================
		向cudp发送数据
		return
			1		成功
			0		失败

		param
		char 	*data	发送数据
		int32_t datalen	数据长度
		int32_t addr_dst 目标地址  
=============================================*/
int32_t YJ_CudpSendTo(YJ_CudpCore * h, char *data, int32_t datalen, int32_t addr_dst);




