/*
 * LM_Cudp_m3.c
 *
 * Created: 2016/9/2 
 *  Author: YJ
 * Email: huangyj@lzrobot.com
 */
 
#include "LM_Cudp_m3.h"
#include "stm32f10x.h"
#include "misc.h"
#include "stm32f10x_can.h"
#include "LM_queue_m3_FreeRtos.h"
#include "SmartTuna.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h" 

struct YJ_CudpCore
{
	int32_t 	IsInit;		
	int32_t 	IsExited;	
	int32_t 	DstAddr_def;//默认目标地址 
	int32_t 	DstAddr;//目标地址 
	int32_t		LocalAddr;//本机地址
	HiQueue		RecvQue;
	HiQueue		SendQue;

	//中断相关
	int32_t		Status_it;
	int32_t		IsHaveItData;
	int32_t		ItDataLen;
	int32_t		SrcAddr; //数据的源地址
	char		*RecvTmp;

	//发送处理相关
	int32_t		Status_spr;
	int32_t		PackLen_spr;
	int32_t		FrameLen_spr;
	int32_t		offset_spr;	  
	int32_t		flag_spr;	//包类型		
	char			SendBuf_spr[8];
	
	SemaphoreHandle_t mutex; 

};

static YJ_CudpCore 	g_core 												= {0};
static char			g_RecvBuf[LM_MAX_CAN_DATA_QUEUE]	= {0};
static char			g_SendBuf[LM_MAX_CAN_DATA_QUEUE]	= {0};
static char			g_RecvTmp[LM_MAX_CAN_PAYLOAD]			= {0};

/****************************************************************************
* 名    称：void RCC_Configuration(void)
* 功    能：系统时钟配置为72MHZ， 外设时钟配置
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/ 
static void RCC_Configuration(void){

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB , ENABLE);
}

/****************************************************************************
* 名    称：void NVIC_Configuration(void)
* 功    能：中断源配置
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;	  
 
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;	   //CAN1 RX0中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;		   //抢占优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			   //子优先级为0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

static void InitCan(int32_t LocalAddr)
{
	CAN_InitTypeDef        	CanInit;
	CAN_FilterInitTypeDef  	CanFilterInit;  
	GPIO_InitTypeDef  		GpioInit;
	
	//LocalAddr = 0x10;
	  RCC_Configuration();						   //系统时钟设置及外设时钟使能	
  	NVIC_Configuration();						   //中断源配置	  
	
	if(1) //最新版硬件使用GPIO_Pin_11 GPIO_Pin_12 作为can管脚 2016.11.24
	{	
		GpioInit.GPIO_Pin 	= GPIO_Pin_11;	                 //PB8:CAN-RX 
		GpioInit.GPIO_Mode 	= GPIO_Mode_IPU;			     //输入上拉
		GPIO_Init(GPIOB, &GpioInit);
		
		GpioInit.GPIO_Pin 		= GPIO_Pin_12;					 //PB9:CAN-TX 
		GpioInit.GPIO_Mode 		= GPIO_Mode_AF_PP;			 //复用模式
		GpioInit.GPIO_Speed 	= GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GpioInit);	
	}
	else
	{
		GPIO_PinRemapConfig(GPIO_Remap1_CAN1 , ENABLE);		     //端口复用为CAN1   
		
		GpioInit.GPIO_Pin 	= GPIO_Pin_8;	                 //PB8:CAN-RX 
		GpioInit.GPIO_Mode 	= GPIO_Mode_IPU;			     //输入上拉
		GPIO_Init(GPIOB, &GpioInit);
		
		GpioInit.GPIO_Pin 		= GPIO_Pin_9;					 //PB9:CAN-TX 
		GpioInit.GPIO_Mode 		= GPIO_Mode_AF_PP;			 //复用模式
		GpioInit.GPIO_Speed 	= GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GpioInit);	
	}

	
	/* CAN寄存器初始化 */
	CAN_DeInit(CAN1);
	CAN_StructInit(&CanInit);
	
	/* CAN单元初始化 */
	CanInit.CAN_TTCM=DISABLE;			   //MCR-TTCM  时间触发通信模式使能
	CanInit.CAN_ABOM=DISABLE;			   //MCR-ABOM  自动离线管理 
	CanInit.CAN_AWUM=DISABLE;			   //MCR-AWUM  自动唤醒模式
	//CAN_InitStructure.CAN_NART=ENABLE;			   //MCR-NART  禁止报文自动重传	  0-自动重传   1-报文只传一次
	CanInit.CAN_NART=DISABLE;			   //MCR-NART  禁止报文自动重传	  0-自动重传   1-报文只传一次
	CanInit.CAN_RFLM=DISABLE;			   //MCR-RFLM  接收FIFO 锁定模式  0-溢出时新报文会覆盖原有报文  1-溢出时，新报文丢弃
	CanInit.CAN_TXFP = ENABLE;
	CanInit.CAN_Mode = CAN_Mode_Normal;
	//CAN_InitStructure.CAN_TXFP=DISABLE;			   //MCR-TXFP  发送FIFO优先级 0-优先级取决于报文标示符 1-优先级取决于发送请求的顺序
	//CAN_InitStructure.CAN_Mode=CAN_Mode_LoopBack;	   //BTR-SILM/LBKM   CAN环回模式
	CanInit.CAN_SJW=CAN_SJW_1tq;		   //BTR-SJW 重新同步跳跃宽度 1个时间单元
	CanInit.CAN_BS1=CAN_BS1_2tq;		   //BTR-TS1 时间段1 占用了2个时间单元
	CanInit.CAN_BS2=CAN_BS2_3tq;		   //BTR-TS1 时间段2 占用了3个时间单元

	//位速率 1Mbps
	CanInit.CAN_Prescaler = 6;			   //BTR-BRP 波特率分频器  定义了时间单元的时间长度 36/(1+2+3)/6=1Mbps
	CAN_Init(CAN1, &CanInit);

	/* CAN过滤器初始化 */
	CanFilterInit.CAN_FilterNumber 	= 0;						//
	CanFilterInit.CAN_FilterMode	= CAN_FilterMode_IdMask;		//FM1R  过滤器组0的工作模式。
																	//0: 过滤器组x的2个32位寄存器工作在标识符屏蔽位模式； 
																	//1: 过滤器组x的2个32位寄存器工作在标识符列表模式。
	CanFilterInit.CAN_FilterScale	= CAN_FilterScale_32bit;		//FS1R 过滤器组0(13～0)的位宽。
																	//0：过滤器位宽为2个16位； 1：过滤器位宽为单个32位。
	
	/* 使能报文标示符过滤器按照标示符的内容进行比对过滤，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */
	CanFilterInit.CAN_FilterIdHigh		= (((u32)LocalAddr << 3) & 0xFFFF0000) >> 16;				//要过滤的ID高位 
	CanFilterInit.CAN_FilterIdLow		= (((u32)LocalAddr << 3) | CAN_ID_EXT | CAN_RTR_DATA) & 0xFFFF;//要过滤的ID低位 
	CanFilterInit.CAN_FilterMaskIdHigh	= 0x3fff;
	CanFilterInit.CAN_FilterMaskIdLow	= 0xffff;
	CanFilterInit.CAN_FilterFIFOAssignment	= 0;					//FFAx : 过滤器位宽设置 报文在通过了某过滤器的过滤后，
																	//将被存放到其关联的FIFO中。 0：过滤器被关联到FIFO0； 1：过滤器被关联到FIFO1。
	CanFilterInit.CAN_FilterActivation		= ENABLE;				//FACTx : 过滤器激活 软件对某位设置1来激活相应的过滤器。只有对FACTx位清0，
																	//或对CAN_FMR寄存器的FINIT位设置1后，才能修改相应的过滤器寄存器
																	//x(CAN_FxR[0:1])。 0：过滤器被禁用； 1：过滤器被激活。
	CAN_FilterInit(&CanFilterInit);
    /* CAN FIFO0 接收中断使能 */ 
  	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
	
	return;		
}

/* =========================================================
		初始化一个cudp对象
		return
			句柄		成功
			NULL	失败

		param
		int32_t DstAddr_def	can  默认目标地址
		int32_t LocalAddr	can   本机地址
===========================================================*/
YJ_CudpCore * YJ_InitCudpObj(int32_t DstAddr_def, int32_t LocalAddr)
{
	YJ_CudpCore *h 	=  &g_core;

	if(h->IsInit)
		return NULL;

	h->mutex = xSemaphoreCreateMutex();  
	InitCan(LocalAddr);
	h->DstAddr_def  = DstAddr_def;
	h->LocalAddr 	  = LocalAddr; 
	h->RecvQue 	= HiCreateQueue(g_RecvBuf, sizeof(g_RecvBuf));
	h->SendQue 	= HiCreateQueue(g_SendBuf, sizeof(g_SendBuf));
	h->IsInit 	= 1;
	h->RecvTmp  = g_RecvTmp;
	return h;
}

/* ============================================
		烧毁一个cudp对象

=============================================*/
void YJ_DeletCudpObj(YJ_CudpCore * h)
{
	if(!h->IsInit)
		return;
	
	h->IsInit = 0;
	CAN_ITConfig(CAN1, CAN_IT_FMP0, DISABLE);
	//HiDestroyQueue(h->RecvQue);
	//HiDestroyQueue(h->SendQue);
	return;
}

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
int32_t YJ_CudpRecv(YJ_CudpCore * h, char *buf, int32_t buflen)
{
	int32_t ret = 0, addr_src;

	xSemaphoreTake( h->mutex, portMAX_DELAY );
	
	do
	{
	 if(!h->IsInit)
		break;

	if(HiGetQueueDataLen(h->RecvQue) < sizeof(int32_t))
		break;

	 HiPeakQueue((char *)&ret,  sizeof(int32_t), 0, h->RecvQue);

	if(HiGetQueueDataLen(h->RecvQue) < (sizeof(int32_t) * 2 + ret))
		break;

	 if(ret > buflen)
	 {
			ret = -1;
		 break;
	 }
	 
	 HiDeQueue((char *)&ret, sizeof(int32_t), h->RecvQue);
	 HiDeQueue((char *)&addr_src, sizeof(int32_t), h->RecvQue);
	 HiDeQueue(buf, ret, h->RecvQue);
	 
	}while(0);
	 
	 xSemaphoreGive(h->mutex);
	
	 return ret;
}

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
int32_t YJ_CudpRecvFrom(YJ_CudpCore * h, char *buf, int32_t buflen, int32_t *addr_src)
{
	int32_t ret = 0;

	xSemaphoreTake( h->mutex, portMAX_DELAY );
	
	do
	{
		if(!h->IsInit)
			break;

		if(HiGetQueueDataLen(h->RecvQue) < sizeof(int32_t))
			break;

		 HiPeakQueue((char *)&ret,  sizeof(int32_t), 0, h->RecvQue);

		if(HiGetQueueDataLen(h->RecvQue) < (sizeof(int32_t) * 2 + ret))
			break;

		 if(ret > buflen)
		 {
			 ret = -1;
			 break;
		 }

		 HiDeQueue((char *)&ret, sizeof(int32_t), h->RecvQue);
		 HiDeQueue((char *)addr_src, sizeof(int32_t), h->RecvQue);
		 HiDeQueue(buf, ret, h->RecvQue);
	}while(0);
	
	xSemaphoreGive(h->mutex);
	
	 return ret;
}

/* ============================================
		向cudp发送数据
		return
			1		成功
			0		失败

		param
		char 	*data	    发送数据
		int32_t datalen	数据长度
		int32_t addr_dst 目标地址  
=============================================*/
int32_t YJ_CudpSendTo(YJ_CudpCore * h, char *data, int32_t datalen, int32_t addr_dst)
{
	int32_t ret = 0;
	
	if(!h->IsInit)
	return 0;

	xSemaphoreTake( h->mutex, portMAX_DELAY );
	
	do{
		if(HiGetQueueIdleNum(h->SendQue) < (datalen + sizeof(int32_t) * 2))
			break;

		HiEnQueue((char *)&datalen, sizeof(int32_t), h->SendQue); /*四字节的长度*/
		HiEnQueue((char *)&addr_dst, sizeof(int32_t), h->SendQue); /*四字节的目标地址*/
		HiEnQueue(data, datalen, h->SendQue); 
		ret = 1;
	}while(0);

	xSemaphoreGive(h->mutex);
	
	return ret;
}

/* ============================================
		向cudp发送数据
		return
			1		成功
			0		失败

		param
		char 	*data	发送数据
		int32_t datalen	数据长度
=============================================*/
int32_t YJ_CudpSend(YJ_CudpCore * h, char *data, int32_t datalen)
{	
	int32_t ret = 0;
	if(!h->IsInit)
		return 0;

		xSemaphoreTake( h->mutex, portMAX_DELAY );
	
	do{
		if(HiGetQueueIdleNum(h->SendQue) < (datalen + sizeof(int32_t) * 2))
			break;

		HiEnQueue((char *)&datalen, sizeof(int32_t), h->SendQue); /*四字节的长度*/
		HiEnQueue((char *)&h->DstAddr_def, sizeof(int32_t), h->SendQue); /*四字节的目标地址*/
		HiEnQueue(data, datalen, h->SendQue);	
		ret = 1;
	}while(0);
	
	xSemaphoreGive(h->mutex);
	
	 return ret;
}

/* ============================================
		cudp从中断接收数据
=============================================*/
static void ProcRecv(YJ_CudpCore * h)
{
	if(!h->IsHaveItData)
		return;
	
	if(HiGetQueueIdleNum(h->RecvQue) < (h->ItDataLen + sizeof(int32_t) * 2))
		return;
	
	HiEnQueue((char *)&h->ItDataLen, sizeof(int32_t), h->RecvQue);
	HiEnQueue((char *)&h->SrcAddr, sizeof(int32_t), h->RecvQue);
	HiEnQueue(h->RecvTmp, h->ItDataLen, h->RecvQue);
	h->IsHaveItData = 0;
  
	return;
}

/* ============================================
		把数据发送到bus上
		return
			1		成功
			0		失败

		param
		in32_t  DstAddr	目标地址
		char 	*data	    发送数据
		int32_t  len	  数据长度
=============================================*/
static int32_t SendCanFrame(int32_t DstAddr, char *data, int32_t len)
{
	int32_t ret;

	CanTxMsg TxMessage;

	/* 发送一帧报文 */
	TxMessage.StdId 	= 0x00;
	TxMessage.ExtId 	= DstAddr;
	TxMessage.IDE 		= CAN_ID_EXT;
	TxMessage.RTR		= CAN_RTR_DATA;
	TxMessage.DLC		= len;
	memcpy(TxMessage.Data, data, len);
	ret = CAN_Transmit(CAN1, &TxMessage);

	if(ret == CAN_TxStatus_NoMailBox)
		return 0;

	return 1;	
}
 

/* ============================================
		cudp把队列的数据发送至fifo
=============================================*/
static void ProcSend(YJ_CudpCore * h)
{
	int32_t ret;
	
	switch(h->Status_spr)
	{
	case 0:			//查询队列是否有数据
		if(HiGetQueueDataLen(h->SendQue) <sizeof(int32_t))
			return;

	  HiPeakQueue((char *)&ret,  sizeof(int32_t), 0, h->SendQue);

		if(HiGetQueueDataLen(h->SendQue) < (sizeof(int32_t) * 2 + ret))
			return;

		h->PackLen_spr = ret;
		h->offset_spr  = 0;	
		h->Status_spr  = 1;
		break;

	case 1:	  		//从队列拿一个can帧的数据
		if(h->offset_spr  == 0) //数据包的开始包
		{		
			HiDeQueue((char *)&ret, sizeof(int32_t), h->SendQue);
			HiDeQueue((char *)&h->DstAddr, sizeof(int32_t), h->SendQue);

			if(h->PackLen_spr <= 8)
			{
				HiDeQueue(h->SendBuf_spr, h->PackLen_spr, h->SendQue);
				h->flag_spr = 2 << 27; //结束包
				h->FrameLen_spr = h->PackLen_spr;
				h->offset_spr	= h->FrameLen_spr;
			}
			else
			{
				HiDeQueue(h->SendBuf_spr, 8, h->SendQue);
				h->flag_spr 	= 0; //开始包
				h->FrameLen_spr = 8;
				h->offset_spr	= h->FrameLen_spr;				
			}	
		}
		else	   
		{
			if(h->PackLen_spr - h->offset_spr <= 8)
			{
 				HiDeQueue(h->SendBuf_spr, h->PackLen_spr - h->offset_spr, h->SendQue);
				h->flag_spr = 2 << 27; //结束包
				h->FrameLen_spr =  h->PackLen_spr - h->offset_spr;
				h->offset_spr	+= h->FrameLen_spr;
			} 
			else
			{
				HiDeQueue(h->SendBuf_spr, 8, h->SendQue);
				h->flag_spr = 1 << 27; //中间包
				h->FrameLen_spr =  8;
				h->offset_spr	+= h->FrameLen_spr;		
			}	
		}
		h->Status_spr  = 2;
		break;
		
	default:
		
		break;
		
	}
	
	if(h->Status_spr == 2)	  //发送到总线上
	{
		if(SendCanFrame(h->DstAddr | h->flag_spr, h->SendBuf_spr, h->FrameLen_spr))
		{
			if(h->flag_spr == 0 || h->flag_spr == (1 << 27)) 
			{
				h->Status_spr  = 1;
				return;
			}
			else
			{
				h->Status_spr  = 0;
				return;
			}
		}
	}
	 return;
}

/* ============================================
		cudp处理循环
=============================================*/
static void YJ_CudpProc(YJ_CudpCore * h)
{
	if(!h->IsInit)
		return;

	 ProcRecv(h);
	 ProcSend(h);
	 return;
}

/* ============================================
		接收中断处理程序
=============================================*/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	CanRxMsg 	RxMessage   =  {0};
	YJ_CudpCore *h 			=  &g_core;
	int32_t		type;
	
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage); //中断标志会被清掉
	
	if(!h->IsInit || h->IsHaveItData)
		return;
		
	//RxMessage.DLC==2) && ((RxMessage.Data[1]RxMessage.ExtId==0x1234
	type      = (RxMessage.ExtId & 0x18000000) >> 27;
	h->SrcAddr = RxMessage.ExtId &  0x3FFFFFF;
	//mDEBUG("id %d", RxMessage.ExtId);
	switch(h->Status_it)
	{
	case 0:  //开始包或结束包
		if(type == 2)
		{
			memcpy(h->RecvTmp, RxMessage.Data, RxMessage.DLC);
		    h->IsHaveItData = 1;
			h->ItDataLen	= RxMessage.DLC;
		}
		else if(type == 1)
		{
			//mERR("recv type err");
			h->ItDataLen = 0;
			break;
			//continue;
		}
		else
		{
			memcpy(h->RecvTmp, RxMessage.Data, RxMessage.DLC);
			h->ItDataLen	= RxMessage.DLC;
			h->Status_it 	= 1;
		}
		break;
	case 1:	//中间包或结束包
		if(type == 0)
		{
			//mERR("recv type err");
			h->ItDataLen	= 0;
			h->Status_it 	= 0;
		}
		else if(type == 1)
		{
			memcpy(h->RecvTmp + h->ItDataLen, RxMessage.Data, RxMessage.DLC);
			h->ItDataLen	+= RxMessage.DLC;
		}
		else
		{
			memcpy(h->RecvTmp + h->ItDataLen, RxMessage.Data, RxMessage.DLC);
			h->ItDataLen	+= RxMessage.DLC;
			h->IsHaveItData	=  1; 
			h->Status_it 	=  0;
		}
		break;
	default:
		break;
	}

   return;
}

static void CudpTask( void *parameters )
{
		YJ_CudpCore * h = &g_core;
	
	  const portTickType xDelay = pdMS_TO_TICKS(3); 
	
		while(!h->IsExited)
		{
			vTaskDelay( xDelay );
			 YJ_CudpProc(h);
		}
		vTaskDelete(NULL);
}

/* =========================================================
		初始化一个cudp
		return
			句柄		成功
			NULL	失败
===========================================================*/
YJ_CudpCore * YJ_CudpInit()
{
	int32_t addr ;
	YJ_CudpCore *h;
	
	if(LM_LOCAL_CAN_ADDR == LMST_HEAD_LOCAL_CTRL_NODE)
	{
		addr = LMST_HEAD_NODE;
	}
	else
	{
		addr = LM_LOCAL_CAN_ADDR;
	}
	
	  h = YJ_InitCudpObj(0x12345, addr);
	
	if(h == NULL)
		return NULL;
	
	xTaskCreate( CudpTask, ( signed portCHAR * ) "CudpTask", 
		configMINIMAL_STACK_SIZE, NULL, 
			LMST_CUDP_TASK_PRIORITY, NULL);
	
	return h;
}

/* =========================================================
		反初始化cudp
===========================================================*/
void YJ_CudpUnInit(YJ_CudpCore * h)
{
	if(!h->IsInit)
		return;
	
	h->IsExited = 1;
	YJ_DeletCudpObj(h);
}



