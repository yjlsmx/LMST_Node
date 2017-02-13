 /*
 * LM_Bootloader.c
 *
 * Created: 2016/9/2 
 *  Author: YJ
 * Email: huangyj@lzrobot.com
 */
 
#include "cJSON.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "LM_Bootloader.h"
#include "SmartTuna.h"
#include "LMST_TailNode.h"
#include "LMST_NodeCommon.h"
#include "LM_Common.h"
#include "OnChip.h"

struct YJ_BtldCore
{

	volatile int32_t 	IsInit;
	uint32_t   				RunMode; //0: bootloader 0xffffffff：用户态 
	uint64_t  				BtldTimeout;
	
	//固件更新相关
	volatile int32_t State_up;
	int32_t					 FileLen_up;	
	int32_t					 BytesOfRecv_up;	
	int32_t					 SerialNum_up;	
	u8							 data_up[FLASH_PAGE_SIZE];

	SemaphoreHandle_t  mutex; 
};

static void SendDataReq(int32_t s)
{
	
	char *tmp = "{\"c\":\"DataReq\",\"n\":%d}";
	char buf[50];
	
	snprintf(buf, 50, tmp, s);
	YJ_SendMsg(buf, 7);
}

static void BtldTask( void *parameters )
{
	int32_t 	cnt = 0;
	uint64_t	ReqTimeout; //请求超时
	int32_t   SerialNum;

	const portTickType xDelay = pdMS_TO_TICKS(20); 
	YJ_BtldCore *h = 	(YJ_BtldCore *)parameters;
	
	YJ_SetTimeOut_ms(&h->BtldTimeout, 3000);
	
	while(1)
	{
		vTaskDelay( xDelay );
		
		if(YJ_IsTimeOut(h->BtldTimeout) && h->RunMode == 0xffffffff)
		{
			YJ_UnInitSystem();
			YJ_IAPJump();		
		}
		
		xSemaphoreTake( h->mutex, portMAX_DELAY );
		
		//程序更新过程监控
		switch(h->State_up)
		{
		case 0:	
			YJ_SetTimeOut_ms(&ReqTimeout, 200);
			SerialNum = h->SerialNum_up;
		break;
		
		case 1:
			if(SerialNum == h->SerialNum_up) 
			{
				if(YJ_IsTimeOut(ReqTimeout))
				{	
					SendDataReq(SerialNum);	
					YJ_SetTimeOut_ms(&ReqTimeout, 200);
					if(cnt++ > 25)  //多次申请都没有数据就终止传输；
					{
						h->State_up = 0;
						cnt 				= 0;
					}
				}
        else
				{
					//等待
				}
			}
			else
			{
				SerialNum = h->SerialNum_up;
				cnt = 0;
				YJ_SetTimeOut_ms(&ReqTimeout, 200);
			}

		break;
		
		}			
		xSemaphoreGive(h->mutex);
	}
	
}

static int32_t GetSerialNum(char *msg)
{
	int32_t ret;
	//网络序到主机序列
	ret = ((msg[0] << 24) + (msg[1] << 16) + (msg[2] << 8) + (msg[3]) );
	
	return ret;
}

static int32_t GetFileLen(char *msg)
{
	int32_t ret = 0;
	
	cJSON *json = NULL , *s = NULL;
	
	//网络序到主机序列
	do{
		 json = cJSON_Parse(msg);
		if (!json)
		{
			mERR("cJSON_Parse fail");
			break;
		}
		else
		{
			s = cJSON_GetObjectItem( json , "value");
			if( s == NULL || s->type != cJSON_Number)
			{
				mERR("have not FileLen");
				break;
			}
			
			ret = s->valueint;
		}
	}while(0);
	
	if(json)
		cJSON_Delete(json);
			
	return ret;
}

/* =========================================================
	擦写一页数据
		return
			1			成功
			0			失败
===========================================================*/
static int32_t  FlashWritePage(uint32_t PageAddr, u8 *data)
{
	FLASH_Status FLASHStatus 	= FLASH_COMPLETE; 
	u16 i 										= 0;
	
	FlashErasePage(PageAddr, 1);
	
	FLASH_Unlock();
	while((i < FLASH_PAGE_SIZE) && (FLASHStatus == FLASH_COMPLETE))
	{
			FLASHStatus = FLASH_ProgramHalfWord(PageAddr, *(u16*)data);
		
			if(FLASHStatus!=FLASH_COMPLETE)
				return 0;
			
			i = i+2;
			PageAddr = PageAddr + 2;
			data = data + 2;
	}
	FLASH_Lock();
	return 1;
}

/* =========================================================
	bootloader处理从上位机接收的信息
===========================================================*/
void YJ_BtldProMsg(YJ_BtldCore *h, char * msg, int32_t len, int32_t type)
{
	u8 i;
	uint32_t PageAddr;
	
	if(h == NULL)
		return;
	
	xSemaphoreTake( h->mutex, portMAX_DELAY );
	
	do{
			switch(h->State_up)
			{
				case 0: //等待更新命令
					
				if(type != 6)  //是否为下载命令
						break;
				
				SendDataReq(h->SerialNum_up);	
				YJ_SetRunMode(0); 
				h->RunMode 	= 0;
				h->FileLen_up 			= GetFileLen(msg);
				h->BytesOfRecv_up	  = 0;
				h->State_up   			= 1; 
				h->SerialNum_up			= 1;
				
				break;
				
				case 1:  //接收文件流
				
				if(type != 8)  //是否为数据
					break;
				
				if(GetSerialNum(msg) != h->SerialNum_up)
					break;
				
				memcpy((h->data_up + (h->BytesOfRecv_up % FLASH_PAGE_SIZE)), (u8 *)(msg + 4), len - 4);
				h->BytesOfRecv_up += len - 4;		
			
				if(h->BytesOfRecv_up != h->FileLen_up)
				{
					if((h->BytesOfRecv_up) % FLASH_PAGE_SIZE == 0)
					{
						PageAddr = YJ_USER_PROGRAM_FLASH_ADDR + h->BytesOfRecv_up - FLASH_PAGE_SIZE;
						if(FlashWritePage(PageAddr, h->data_up) != 1)
						{
							mERR("FlashWritePage error !SerialNum %d", h->SerialNum_up);
						}
					}
					
					h->SerialNum_up++;
					SendDataReq(h->SerialNum_up);					
				}	
				else
				{
					if(h->BytesOfRecv_up % FLASH_PAGE_SIZE == 0)
					{
						PageAddr = YJ_USER_PROGRAM_FLASH_ADDR + h->BytesOfRecv_up - FLASH_PAGE_SIZE;
					}
					else
					{
						PageAddr = YJ_USER_PROGRAM_FLASH_ADDR + h->BytesOfRecv_up - (h->BytesOfRecv_up % FLASH_PAGE_SIZE);
					}
					
					if(FlashWritePage(PageAddr, h->data_up) != 1)
					{
						mERR("FlashWritePage error !SerialNum %d", h->SerialNum_up);
					}
					
					YJ_SendMsg("{\"c\":\"UpdateFinish\"}", 9);
					YJ_SetTimeOut_ms(&h->BtldTimeout, 1500);
					h->State_up = 0;
					YJ_SetRunMode(0xffffffff); 
					h->RunMode 	= 0xffffffff;
				}
			
				break;
			}
	}while(0);
	
	xSemaphoreGive(h->mutex);
	
}

/* =========================================================
		初始化bootloader模块
		return
			句柄		成功
			NULL	  失败
===========================================================*/
YJ_BtldCore * YJ_BtldInit(void)
{
	YJ_BtldCore *h = (YJ_BtldCore *)pvPortMalloc(sizeof(struct YJ_BtldCore));

	if(h == NULL)
		return NULL;

	memset(h, 0, sizeof(struct YJ_BtldCore));
	h->mutex = xSemaphoreCreateMutex();  
	
		xTaskCreate( BtldTask, ( signed portCHAR * ) "BtldTask", 
	configMINIMAL_STACK_SIZE, (void*)h, 
		LMST_BTLD_TASK_PRIORITY, NULL);

	h->IsInit  = 1;
	h->RunMode = YJ_GetRunMode();
	return h;
	
}

void YJ_BtldUnInit(YJ_BtldCore * h)
{
	
}




