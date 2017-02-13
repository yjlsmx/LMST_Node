/*
 * LMST_HeadNode.c
 *
 * Created: 2016/9/2 
 *  Author: YJ
 * Email: huangyj@lzrobot.com
 */
 
#include "cJSON.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "SmartTuna.h"
#include "LMST_HeadNode.h"
#include "LMST_NodeCommon.h"

//摄像头led灯
void YJ_InitLed()
{	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;				//默认OD门复用输出，需上拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;				//GPIO速度10MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	//GPIO_SetBits(GPIOA,GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
	GPIO_ResetBits(GPIOA,GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
}

static void ReplyHeatBeat(char *msg)
{
	char *tmp = "{\"c\":\"HeartBeat\",\"n\":0,\"s\":2,\"temp\":%d,\"humidity\":%d}";
	char buf[240];
	
	snprintf(buf, 240, tmp, 0, 0);
	YJ_SendMsg(buf, 3);
}
		
static void ProcMsg(char *msg)
{
		cJSON *json = NULL , *s = NULL;
						
	 do{
			// 解析数据包
			//json = cJSON_Parse("{\"c\":\"SetMotorSpeed_l\",\"n\":0,\"value\":7}");
		 json = cJSON_Parse(msg);
			if (!json)
			{
				mERR("cJSON_Parse fail");
				break;
			}
			else
			{
				s = cJSON_GetObjectItem( json , "c");
				if( s == NULL || s->type != cJSON_String)
				{
					mERR("have not c");
					break;
				}
				
				if(strcmp(s->valuestring, "OpenLight") == 0)
				{
					GPIO_SetBits(GPIOA,GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);			
				}
				else if(strcmp(s->valuestring, "CloseLight") == 0)
				{
					GPIO_ResetBits(GPIOA,GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
				}
				else if(strcmp(s->valuestring, "HeartBeat") == 0)
				{
					ReplyHeatBeat(msg);
				}
				else if(strcmp(s->valuestring, "Reboot") == 0)
				{
					mDEBUG("Reboot");
					YJ_UnInitSystem();
					YJ_ReBoot();
				}
				else
				{
					mERR("error c");
					break;
				}
				
				//mDEBUG("c: %d", s->valueint);
			}
			
		}while(0);
	 
		if(json)
		 cJSON_Delete(json);
}


void YJ_HeadNodeTask( void *parameters ) 
{
		int32_t 	ret = 1; 
		char       * tmp = pvPortMalloc(LM_MAX_CAN_PAYLOAD);
	  int32_t   SrcAddr;
		int32_t   type;
	
	  const portTickType xDelay = pdMS_TO_TICKS(3);  //宏pdMS_TO_TICKS用于将毫秒转成节拍数
	  mDEBUG("boot ----> bootloader");
		while(1)
		{
			vTaskDelay( xDelay );
			
			//GPIO_SetBits(GPIOA,GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);		
			if((ret = YJ_UudpRecv(g_UudpCore, tmp, LM_MAX_CAN_PAYLOAD)) > 0)
			{
				if(YJ_AplGetAddr(tmp) == LM_LOCAL_CAN_ADDR)
				{
					type = YJ_AplGetType(tmp);
					if(type == 6 || type == 8)
					{
						YJ_BtldProMsg(g_BtldCore, tmp + 8, ret - 8, type);
					}
					else
					{
						ProcMsg(tmp + 8);//除去8字节的应用层头
					}	
				}
				else
				{
					YJ_CudpSendTo(g_CudpCore, tmp, ret, YJ_AplGetAddr(tmp));
				}
				//YJ_SendMsg("ok\n");
			}

			if((ret = YJ_CudpRecvFrom(g_CudpCore, tmp, LM_MAX_CAN_PAYLOAD, &SrcAddr)) > 0)
			{
				YJ_UudpSend(g_UudpCore, tmp, ret);					
			}
			 
		}
}