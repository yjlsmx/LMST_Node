 /*
 * LMST_NodeCommon.c
 *
 * Created: 2016/9/2 
 *  Author: YJ
 * Email: huangyj@lzrobot.com
 */
 
#include "stm32f10x_iwdg.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "SmartTuna.h"
#include "LMST_NodeCommon.h"
#include "LM_BatteryManager.h"	
#include "LMST_HeadNode.h"
#include "LM_Bootloader.h"
#include "LM_Mpu.h"
#include "LM_BuoyantDevices.h"

YJ_MtctCore 			* g_MtctCore;
YJ_CudpCore 			* g_CudpCore;
YJ_UudpCore 			* g_UudpCore;
YJ_SysClockCore 	* g_SysClockCore;
YJ_BtldCore 			* g_BtldCore;
YJ_MpuCore 				* g_MpuCore;
YJ_ButdCore 			* g_ButdCore;

char g_NodeCommonBuff[LM_MAX_CAN_PAYLOAD + 128];

//跳转到用户态
void YJ_IAPJump(void)
{
//	TIM_Cmd(TIM1, DISABLE);
//	TIM_Cmd(TIM2, DISABLE);
//	TIM_Cmd(TIM3, DISABLE);
	__set_MSP(*(__IO uint32_t*)(0x8008000));
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x8000);
	__disable_irq();
	//YJ_IwdgInit();
	((void(*)())(*(vu32*)0x8008004))();
}

//重启
void YJ_ReBoot(void)
{
//	__set_MSP(*(__IO uint32_t*)(0x8000000));
//	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);
//	__disable_irq();
//	((void(*)())(*(vu32*)0x8000004))();
	
	__disable_irq();
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);    //允许写IWDG  
	IWDG_SetPrescaler(IWDG_Prescaler_64);    //设置分频  
	IWDG_SetReload(1);       //设置Reload  
	IWDG_ReloadCounter();//重载值  
	IWDG_Enable();//使能IDWG 
	while(1);
	
}

// 喂狗
static void IwdgTask( void *parameters )
{
	const portTickType xDelay = pdMS_TO_TICKS(100);
	
	while(1)
	{
		vTaskDelay( xDelay );
		IWDG_ReloadCounter();
	}
}

//看门狗初始化 TimeOut = （64/(40*10^3)）*308 =0.5s 
void YJ_IwdgInit()
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);    //允许写IWDG  
	IWDG_SetPrescaler(IWDG_Prescaler_64);    //设置分频  
	IWDG_SetReload(616);       //设置Reload  
	IWDG_ReloadCounter();//重载值  
	IWDG_Enable();//使能IDWG 
	
	xTaskCreate( IwdgTask, ( signed portCHAR * ) "IwdgTask", 
	128, NULL, 
	LMST_WDG_TASK_PRIORITY, NULL);
}

//系统初始化
void YJ_InitSystem()
{
	//设置中断向量
	//NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x8000);
	//__enable_irq();

	#if _LMST_NODE_TYPE == LMST_HEAD_NODE
		YJ_IwdgInit();
		g_SysClockCore  	= YJ_SysClockInit();
		g_CudpCore 				= YJ_CudpInit();
		YJ_InitLed();
		g_UudpCore 				= YJ_UudpInit(230400);//YJ_UudpInit(115200);
		g_BtldCore				= YJ_BtldInit();
	#elif _LMST_NODE_TYPE == LMST_HEAD_LOCAL_CTRL_NODE
	 YJ_IwdgInit();
		g_SysClockCore  	= YJ_SysClockInit();
		g_CudpCore 				= YJ_CudpInit();
		YJ_InitLed();
		g_UudpCore 				= YJ_UudpInit(230400);//YJ_UudpInit(115200);
		g_BtldCore				= YJ_BtldInit();
	#elif _LMST_NODE_TYPE == LMST_TAIL_NODE
		YJ_IwdgInit();
		g_SysClockCore  	= YJ_SysClockInit();
		g_MtctCore 				= YJ_MtctInit();
		g_CudpCore 				= YJ_CudpInit();
		g_BtldCore				= YJ_BtldInit();
		YJ_BtmgInit();
	#elif _LMST_NODE_TYPE == LMST_SERVO_MOTOR_NODE
		YJ_IwdgInit();
		g_SysClockCore  	= YJ_SysClockInit();
		g_MtctCore 				= YJ_MtctInit();
		g_CudpCore 				= YJ_CudpInit();
		g_BtldCore				= YJ_BtldInit();
	#elif _LMST_NODE_TYPE == LMST_SENSOR_NODE
		YJ_IwdgInit();
		g_SysClockCore  	= YJ_SysClockInit();
		g_CudpCore 				= YJ_CudpInit();
		g_MpuCore					= YJ_MpuInit();
		g_BtldCore				= YJ_BtldInit();
	#elif _LMST_NODE_TYPE == LMST_DIVING_NODE
		YJ_IwdgInit();
		g_SysClockCore  	= YJ_SysClockInit();
		g_CudpCore 				= YJ_CudpInit();
		g_BtldCore				= YJ_BtldInit();
		g_ButdCore				= YJ_ButdInit();
	#endif
	
}

//系统反初始化
void YJ_UnInitSystem()
{
	#if _LMST_NODE_TYPE == LMST_HEAD_NODE
		YJ_BtldUnInit(g_BtldCore);
		YJ_SysClockUninit(g_SysClockCore);
		YJ_CudpUnInit(g_CudpCore);
		YJ_UudpUninit(g_UudpCore);
	#elif _LMST_NODE_TYPE == LMST_HEAD_LOCAL_CTRL_NODE
		YJ_BtldUnInit(g_BtldCore);
		YJ_SysClockUninit(g_SysClockCore);
		YJ_CudpUnInit(g_CudpCore);
		YJ_UudpUninit(g_UudpCore);
	#elif _LMST_NODE_TYPE == LMST_TAIL_NODE
		YJ_BtldUnInit(g_BtldCore);
		YJ_SysClockUninit(g_SysClockCore);
		YJ_MtctUnInit(g_MtctCore);
		YJ_CudpUnInit(g_CudpCore);
		YJ_BtmgUnInit();
	#elif _LMST_NODE_TYPE == LMST_SERVO_MOTOR_NODE
		YJ_BtldUnInit(g_BtldCore);
		YJ_SysClockUninit(g_SysClockCore);
		YJ_MtctUnInit(g_MtctCore);
		YJ_CudpUnInit(g_CudpCore);
	#elif _LMST_NODE_TYPE == LMST_SENSOR_NODE
		YJ_BtldUnInit(g_BtldCore);
		YJ_SysClockUninit(g_SysClockCore);
		YJ_CudpUnInit(g_CudpCore);
		//YJ_MpuUnInit(g_MpuCore);
	#elif _LMST_NODE_TYPE == LMST_DIVING_NODE
		YJ_ButdUnInit(g_ButdCore);
		YJ_BtldUnInit(g_BtldCore);
	  YJ_CudpUnInit(g_CudpCore);
		YJ_SysClockUninit(g_SysClockCore);
	#endif
	
	taskENTER_CRITICAL(); //挂起操作系统，关闭中断
}

/* ============================================
    从应用层数据包中获取can地址
		return
			can地址
=============================================*/
int32_t YJ_AplGetAddr(char *tmp)
{
	int32_t addr = 0;
	
	//网络序到主机序列
	addr = ((tmp[4] << 24) + (tmp[5] << 16) + (tmp[6] << 8) + (tmp[7]) );
	
	return addr;
}

/* ============================================
    从应用层数据包中获取包类型
		return
			包类型
=============================================*/
int32_t YJ_AplGetType(char *tmp)
{
	int32_t t = 0;
	
	//网络序到主机序列
	t = ((tmp[0] << 24) + (tmp[1] << 16) + (tmp[2] << 8) + (tmp[3]) );
	
	return t;
}

/* ============================================
    封装应用层协议
		return
			数据长度		
=============================================*/
int32_t YJ_PackedApl(char *buf, int32_t type,char *data, int32_t datalen)
{
		int32_t 	i     = 0;
	  int32_t 	addr  = LM_LOCAL_CAN_ADDR;
		
		buf[i++] = (type & 0xff000000) >> 24; //从主机序（小端） 到 网络序（大端） 
		buf[i++] = (type & 0xff0000)   >> 16;
	  buf[i++] = (type & 0xff00)     >> 8;
		buf[i++] = (type & 0xff);
					
		buf[i++] = (addr & 0xff000000) >> 24; //从主机序（小端） 到 网络序（大端） 
		buf[i++] = (addr & 0xff0000)   >> 16;
	  buf[i++] = (addr & 0xff00)     >> 8;
		buf[i++] = (addr & 0xff);
		
		memcpy(buf + i, data, datalen);
		return datalen + i;
}

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
int32_t YJ_RecvMsg( char *buf, int32_t buflen, int32_t *type)
{
	int32_t 	ret = 0; 
	char      tmp[LM_MAX_CAN_PAYLOAD];
	
	#if _LMST_NODE_TYPE == LMST_HEAD_NODE
			//头舱节点只做透传不做数据接收；
		//ret = YJ_UudpRecv(g_UudpCore, tmp, LM_MAX_CAN_PAYLOAD);
	#else
	ret = YJ_CudpRecv(g_CudpCore, tmp, LM_MAX_CAN_PAYLOAD);
	#endif
	
	if(ret > 0)
	{
		if(ret > buflen)
			return -1;
		
		*type = YJ_AplGetType(tmp);
		memcpy(buf, tmp + 8, ret - 8);
	  return ret - 8;
	}
	
	return ret;
}

/* ============================================
		向上位机发送消息(ascii)
		return
			1		成功
			0		失败

		param
		char 	*msg	发送数据
=============================================*/
int32_t YJ_SendMsg(char *msg, int32_t type)
{
		int32_t 	ret; 
		int32_t 	datalen     = strlen(msg) + 1;
	  char      tmp[LM_MAX_CAN_PAYLOAD];
	
	if(datalen > LM_MAX_CAN_PAYLOAD - 8)
		return 0;
		
		ret = YJ_PackedApl(tmp, type, msg, datalen);
		
	#if _LMST_NODE_TYPE == LMST_HEAD_NODE
	return YJ_UudpSend(g_UudpCore, tmp, ret);	
	#else
	return YJ_CudpSendTo(g_CudpCore, tmp, ret, LMST_HEAD_NODE);	
	#endif
	
}

/* ============================================
		向上位机发送调试信息(ascii)
		return
			1		成功
			0		失败

		param
		char 	*msg	发送数据
=============================================*/
int32_t YJ_DEBUG(char *msg)
{
		int32_t 	ret; 
		int32_t 	type  			= 4;
		int32_t 	datalen     = strlen(msg) + 1;
	
	  char      tmp[LM_MAX_CAN_PAYLOAD];
	
		if(datalen > LM_MAX_CAN_PAYLOAD - 8)
			return 0;
		
		ret = YJ_PackedApl(tmp, type, msg, datalen);
		
	#if (_LMST_NODE_TYPE == LMST_HEAD_NODE || _LMST_NODE_TYPE == LMST_HEAD_LOCAL_CTRL_NODE)
	return YJ_UudpSend(g_UudpCore, tmp, ret);	
	#else
	return YJ_CudpSendTo(g_CudpCore, tmp, ret, LMST_HEAD_NODE);	
	#endif
		
}
							

/* ============================================
		向上位机发送错误信息(ascii)
		return
			1		成功
			0		失败

		param
		char 	*msg	发送数据
=============================================*/
int32_t YJ_ERR(char *msg)
{
		int32_t 	ret; 
		int32_t 	type  			= 5;
		int32_t 	datalen     = strlen(msg) + 1;
	
	  char      tmp[LM_MAX_CAN_PAYLOAD];
	
		if(datalen > LM_MAX_CAN_PAYLOAD - 8)
			return 0;
		
		ret = YJ_PackedApl(tmp, type, msg, datalen);
	
	#if (_LMST_NODE_TYPE == LMST_HEAD_NODE || _LMST_NODE_TYPE == LMST_HEAD_LOCAL_CTRL_NODE)
	return YJ_UudpSend(g_UudpCore, tmp, ret);	
	#else
	return YJ_CudpSendTo(g_CudpCore, tmp, ret, LMST_HEAD_NODE);	
	#endif
		
}
