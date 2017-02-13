
/*
 * LMST_DivingNode.c
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
#include "LMST_NodeCommon.h"
#include "LM_Mpu.h"
#include "LMST_DivingNode.h"
#include "LM_BuoyantDevices.h"

static void ReplyHeatBeat(char *msg)
{
	char *tmp = "{\"c\":\"HeartBeat\",\"n\":0,\"s\":1,\"DivingState\":%d}";
	char buf[240];
	
	snprintf(buf, 240, tmp, GetButdState(g_ButdCore));
	YJ_SendMsg(buf, 3);
}
		
static void ProcMsg(char *msg)
{
		cJSON *json = NULL , *s = NULL, *v = NULL;
						
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
				
				v = cJSON_GetObjectItem( json , "value");
				
				if(strcmp(s->valuestring, "SetDivingState") == 0)
				{
					SetButdState(g_ButdCore, v->valueint);
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

void YJ_DivingNodeTask( void *parameters )
{
		int32_t 	ret = 1; 
		char       * msg = pvPortMalloc(LM_MAX_CAN_PAYLOAD);
	  int32_t   type;
	
	  const portTickType xDelay = pdMS_TO_TICKS(3);  //宏pdMS_TO_TICKS用于将毫秒转成节拍数
	  mDEBUG("boot ----> user");
		while(1)
		{
			vTaskDelay( xDelay );

			if((ret = YJ_RecvMsg( msg, LM_MAX_CAN_PAYLOAD, &type)) > 0)
			{
				if(type == 6 && type == 8)
				{
					//
				}
				else
				{
					ProcMsg(msg);
				}

			 //mDEBUG( msg);				
			}
		}
}