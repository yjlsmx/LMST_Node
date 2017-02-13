#pragma once

#include <stdio.h>
#include <stdlib.h>
#include "stm32f10x.h"
#include "core_cm3.h"
#include "LM_Cudp_m3.h"

extern char g_NodeCommonBuff[LM_MAX_CAN_PAYLOAD + 128];

#define mERR(fmt, args...) do{vTaskSuspendAll(); \
												snprintf(g_NodeCommonBuff,LM_MAX_CAN_PAYLOAD + 128, "%s:%d: "fmt"\n",  __func__, __LINE__,## args); \
												YJ_ERR(g_NodeCommonBuff); \
												xTaskResumeAll(); \
												}while(0);

#define mDEBUG(fmt, args...) do{vTaskSuspendAll(); \
												snprintf(g_NodeCommonBuff,LM_MAX_CAN_PAYLOAD + 128, "%s:%d: "fmt"\n",  __func__, __LINE__,## args); \
												YJ_DEBUG(g_NodeCommonBuff); \
												xTaskResumeAll(); \
												}while(0);
																		
//系统初始化
void YJ_InitSystem(void);
									
//系统反初始化
void YJ_UnInitSystem(void);
												
//重启
void YJ_ReBoot(void);
									
/* ============================================
    接收上位机发送的消息(ascii)
		return
			数据长度		成功
			0			没有
			-1			缓存长度太短

		param
		char 	*buf	数据缓存
		int32_t buflen	缓存长度
=============================================*/
int32_t YJ_RecvMsg( char *buf, int32_t buflen, int32_t *type);

/* ============================================
		向上位机发送消息(ascii)
		return
			1		成功
			0		失败

		param
		char 	*msg	发送数据
=============================================*/
int32_t YJ_SendMsg(char *msg, int32_t type);

/* ============================================
		向上位机发送调试信息(ascii)
		return
			1		成功
			0		失败

		param
		char 	*msg	发送数据
=============================================*/
int32_t YJ_DEBUG(char *msg);

/* ============================================
		向上位机发送错误信息(ascii)
		return
			1		成功
			0		失败

		param
		char 	*msg	发送数据
=============================================*/
int32_t YJ_ERR(char *msg);

/* ============================================
    封装应用层协议
		return
			数据长度		
=============================================*/
int32_t YJ_PackedApl(char *buf, int32_t type,char *data, int32_t datalen);

/* ============================================
    从应用层数据包中获取can地址
		return
			can地址
=============================================*/
int32_t YJ_AplGetAddr(char *tmp);

/* ============================================
    从应用层数据包中获取包类型
		return
			包类型
=============================================*/
int32_t YJ_AplGetType(char *tmp);

//看门狗初始化 TimeOut = （64/(40*10^3)）*308 =0.5s 
void YJ_IwdgInit();

//跳转到用户态
void YJ_IAPJump(void);
