/*
 * LM_Uudp.c
 *
 * Created: 2016/9/2 
 *  Author: YJ
 * Email: huangyj@lzrobot.com
 */ 
 
#include "LM_Uudp.h"
#include "LM_queue_m3_FreeRtos.h"
#include "LT_log.h"
#include "SmartTuna.h"
#include "LM_Common.h"

#include "stm32f10x_usart.h"
#include "stm32f10x.h"
#include "misc.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h" 
	
struct YJ_UudpCore
{
	int32_t 	IsInit;		
	int32_t 	IsExited;	
	HiQueue		RecvQue;
	
	//队列缓存
	uint8_t RecvBuf_que[LM_MAX_UUDP_DATA_QUEUE];
	
	//接收中断相关
	int32_t		Status_it;
	//int32_t		IsHaveItData;
	int32_t		ItDataLen;
	int32_t		RecvPackLen;
	uint64_t  TimeOut_it;
	uint8_t		RecvTmp[LM_MAX_UUDP_PAYLOAD];

	//发送中断相关
	int32_t		offset_spr;	  
	int32_t		PackLen_spr;
	uint8_t		SendBuf_spr[LM_MAX_UUDP_PAYLOAD];
	
	SemaphoreHandle_t mutex; 	
	
};

static YJ_UudpCore * g_UudpCore;

void UARTInit(int32_t BaudRate)
{
	USART_InitTypeDef USART_InitStructure;               			//串口设置默认参数声明
	GPIO_InitTypeDef GPIO_InitStructure;							//GPIO设置默认参数声明
	NVIC_InitTypeDef NVIC_InitStructure;
   
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 , ENABLE);			//RCC时钟设置，串口1时钟开启

	USART_InitStructure.USART_BaudRate = BaudRate;                    //串口2 波特率115200 
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; 	//串口2 字长8位 
	USART_InitStructure.USART_StopBits = USART_StopBits_1;			//串口2 1位停止字节 
	USART_InitStructure.USART_Parity = USART_Parity_No;				//串口2 无奇偶校验 
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//串口2 无流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//串口2 打开Rx接收和Tx发送功能 
	USART_Init(USART2, &USART_InitStructure);						//串口2参数初始化
	USART_ClearITPendingBit(USART2, USART_IT_TC);                   //串口2清空发送中断
	USART_ClearITPendingBit(USART2, USART_IT_RXNE);                 //串口2清空接收
	USART_ITConfig(USART2, USART_IT_TC, ENABLE);					//串口2发送中断开启
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);                  //串口2接收中断开启
  USART_Cmd(USART2, ENABLE);
    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;                       //PA2 TX2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;					//默认OD门复用输出，需上拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;				//GPIO速度10MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);                          //TX初始化 
    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;                     	//PA3 RX2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 			//浮空输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;				//GPIO速度10MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);                          //RX初始化
	
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;				
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY; 		
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;				
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					
	NVIC_Init(&NVIC_InitStructure);		
	
}


/* =========================================================
		uudp协议校验和
				return
					1				正确
					0				错误
===========================================================*/
static int32_t Checksum(char *data, int32_t len)
{
	int32_t i;
	char sum = 0;
	
	for(i = 4; i < len - 3; i++)
	{
		sum += data[i];
	}
	
	if(sum != data[len - 3])
		return 0;
	
	return 1;
	
}

/* =========================================================
		Uudp接收处理
===========================================================*/
static void PrcRecv_it(YJ_UudpCore *h)
{
	
	if(HiGetQueueIdleNum(h->RecvQue) < (h->RecvPackLen + sizeof(int32_t)))
		return;
	
	HiEnQueue_nolock((char *)&h->RecvPackLen, sizeof(int32_t), h->RecvQue);
	HiEnQueue_nolock((h->RecvTmp + 4), h->RecvPackLen, h->RecvQue);
  
	return;
}

/* =========================================================
		中断中的接收处理函数
===========================================================*/
static void ItRecvPrc(YJ_UudpCore * h, char data)
{
	
	h->RecvTmp[h->ItDataLen++] = data;
	
	switch(h->Status_it)
	{
		case 0: //等待包头
			
			if(h->ItDataLen != 2)
				break;
			
			if(h->RecvTmp[0] != (uint8_t)0xfa || h->RecvTmp[1] != (uint8_t)0xfb) //判断包头标识
			{
				h->RecvTmp[0] = h->RecvTmp[1];
				h->ItDataLen = 1;
				 break;
			}
			
			h->Status_it = 1;					
			break;
			
		case 1:  //获取包长
			
			if( h->ItDataLen != 4)
				break;
			
			//从网络序（大端） 到主机序（小端）
			h->RecvPackLen = h->RecvTmp[2];
			h->RecvPackLen = h->RecvPackLen << 8;
			h->RecvPackLen += h->RecvTmp[3];
			
			if(h->RecvPackLen > LM_MAX_UUDP_PAYLOAD) 
			{
				h->RecvTmp[0] = h->RecvTmp[2];
				h->RecvTmp[1] = h->RecvTmp[3];
				h->ItDataLen    = 2;
				h->Status_it    = 0;
				break;
			}
			
			YJ_SetTimeOut_ms(&h->TimeOut_it, 100);
			h->Status_it = 2;				
			break;
			
		case 2:
			
			if(YJ_IsTimeOut(h->TimeOut_it))
			{
				h->ItDataLen    = 0;
				h->Status_it    = 0;
				break;
			}
				
			if( h->ItDataLen != h->RecvPackLen + 7) 
				break;
			
			if((h->RecvTmp[h->ItDataLen - 2] != 0xfc) || 
				(h->RecvTmp[h->ItDataLen - 1] != 0xfd)  ||
			   !Checksum(h->RecvTmp, h->ItDataLen)) 
			{
				h->ItDataLen    = 0;
				h->Status_it    = 0;
				break;
			}
			
			h->Status_it    = 0;
			h->ItDataLen    = 0;
			PrcRecv_it(g_UudpCore);
			//h->IsHaveItData = 1;
			break;
			
		default:
			
			break;
	}
}

/* =========================================================
		接收和发送中断函数
===========================================================*/
void vUARTInterruptHandler( void )
{
		YJ_UudpCore * h = g_UudpCore;
	char RXBuffer;
	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)		
	{
		RXBuffer = USART_ReceiveData(USART2);	
		ItRecvPrc(h, RXBuffer);
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		
	}
	
	if( USART_GetITStatus( USART2, USART_IT_TXE ) == SET )
	{
		USART_SendData( USART2, h->SendBuf_spr[h->PackLen_spr - h->offset_spr--] );
		
		if(h->offset_spr == 0)
		{
			USART_ITConfig( USART2, USART_IT_TXE, DISABLE );
		}
	}
	
	if (USART_GetITStatus(USART2, USART_IT_TC) != RESET)		
	{
        USART_ClearITPendingBit(USART2, USART_IT_TC);           //清除中断标志
	}
}


/* =========================================================
		初始化Uudp
		return
			句柄		成功
			NULL	失败

			int32_t BaudRate 波特率
===========================================================*/
YJ_UudpCore * YJ_UudpInit(int32_t BaudRate)
{
	YJ_UudpCore *h = (YJ_UudpCore *)pvPortMalloc(sizeof(struct YJ_UudpCore));
	
	if(h == NULL)
		return NULL;
	
	memset(h, 0, sizeof(struct YJ_UudpCore));
	h->mutex    = xSemaphoreCreateMutex();  
	h->RecvQue 	= HiCreateQueue(h->RecvBuf_que , sizeof(h->RecvBuf_que));
	
	UARTInit(BaudRate);	
  h->IsInit  = 1;	
	g_UudpCore = h;
	
	return h;
}

/* ============================================
		反初始化Uudp
=============================================*/
void YJ_UudpUninit(YJ_UudpCore * h)
{
	
	if(h == NULL)
		return;
	
	h->IsExited = 1;
	
	USART_Cmd(USART2, DISABLE);
	USART_ITConfig(USART2, USART_IT_TC, DISABLE);					
	USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
	
}

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
int32_t YJ_UudpRecv(YJ_UudpCore * h, char *buf, int32_t buflen)
{
	int32_t ret = 0;

	xSemaphoreTake( h->mutex, portMAX_DELAY );
	do{
		if(!h->IsInit)
			break;

		if(HiGetQueueDataLen(h->RecvQue) < sizeof(int32_t))
			break;

		 HiPeakQueue((char *)&ret,  sizeof(int32_t), 0, h->RecvQue);

		if(HiGetQueueDataLen(h->RecvQue) < (sizeof(int32_t) + ret))
			break;

		 if(ret > buflen)
		 {
			 ret = -1;
			 break;
		 }

		 HiDeQueue_nolock((char *)&ret, sizeof(int32_t), h->RecvQue);
		 HiDeQueue_nolock(buf, ret, h->RecvQue);
		 
	}while(0);
	xSemaphoreGive(h->mutex);
	
	 return ret;
}

/* ============================================
		向Uudp发送数据
		return
			1		成功
			0		失败

		param
		char 	*data	发送数据
		int32_t datalen	数据长度
=============================================*/
int32_t YJ_UudpSend(YJ_UudpCore * h, char *data, int32_t datalen)
{
	int32_t ret = 0, i = 0, j;
	char sum = 0;
	
	xSemaphoreTake( h->mutex, portMAX_DELAY );
	do
	{
		if(h->offset_spr != 0 || datalen > sizeof(h->SendBuf_spr))
			break;
		
		//封包
		h->PackLen_spr 		= datalen + 7;
		h->offset_spr  		= datalen + 7;
		h->SendBuf_spr[i++] = 0xfa; 
		h->SendBuf_spr[i++] = 0xfb; 
		h->SendBuf_spr[i++] = (datalen & 0xff00) >> 8; //从主机序（小端） 到 网络序（大端） 
		h->SendBuf_spr[i++] = (datalen & 0xff); 
		memcpy(h->SendBuf_spr + 4, data, datalen);
		i += datalen;
		for(j = 0; j < datalen; j++)
		{
			sum += data[j]; 
		}
		h->SendBuf_spr[i++] = sum;
		h->SendBuf_spr[i++] = 0xfc; 
		h->SendBuf_spr[i]   = 0xfd; 
		
		USART_ITConfig( USART2, USART_IT_TXE, ENABLE );
		
		while(h->offset_spr != 0){};
	}while(0);
	
	xSemaphoreGive(h->mutex);
	
	return ret;
}




				      
			       