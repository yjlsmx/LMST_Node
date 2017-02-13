/*
 * LMST_TailNode.c
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
#include "LMST_TailNode.h"
#include "LMST_NodeCommon.h"
#include "LM_BatteryManager.h"	
#include "LM_Common.h"

static void ReplyHeatBeat(char *msg)
{
	char *tmp = "{\"c\":\"HeartBeat\",\"n\":0,\"s\":2,\"Speed\":%d,\"Direct\":%d,\"temp\":%d,\"humidity\":%d,\"electric\":%d}";
	char buf[240];
	YJ_MtctParam p = {0};
	
	YJ_MtctGetParam(g_MtctCore, &p);
	snprintf(buf, 240, tmp, p.Speed, p.Direct,  0, 0, YJ_BtmgGetParam());
	YJ_SendMsg(buf, 3);
}

static void ProcMsg(char *msg)
{
		cJSON *json = NULL , *s = NULL, *v = NULL;
		YJ_MtctParam p = {0};
		YJ_MtctGetParam(g_MtctCore, &p);
						
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
				
				if(strcmp(s->valuestring, "SetSpeed") == 0)
				{
					p.Speed = v->valueint;				
				}
				else if(strcmp(s->valuestring, "SetDirection") == 0)
				{
					p.Direct = v->valueint;	
				}
				else if(strcmp(s->valuestring, "SetTraiFinOffset") == 0)
				{
					p.ServoOffset[0] = v->valueint;	
					p.ServoOffset[1] = v->valueint;
					p.ServoOffset[2] = v->valueint;
				}
				else if(strcmp(s->valuestring, "SaveTraiFinOffset") == 0)
				{
					YJ_SaveTraiFinOffset(g_MtctCore);
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
				
				YJ_MtctSetParam(g_MtctCore, &p);
				//mDEBUG("c: %d", s->valueint);
			}
			
		}while(0);
	 
		if(json)
		 cJSON_Delete(json);
}

void ProcUpdataFirmWare(ret, msg)
{
	
}

void YJ_TailNodeTask( void *parameters )
{
		int32_t 	ret = 1; 
		char       * msg = pvPortMalloc(LM_MAX_CAN_PAYLOAD);
		int32_t   type;
	
	  const portTickType xDelay = pdMS_TO_TICKS(5);  //宏pdMS_TO_TICKS用于将毫秒转成节拍数
	  mDEBUG("boot ----> bootloader");
	while(1)
	{
		vTaskDelay( xDelay );

		if((ret = YJ_RecvMsg( msg, LM_MAX_CAN_PAYLOAD, &type)) > 0)
		{
			//mDEBUG("type %d", type);
			if(type == 6 || type == 8)
			{
				YJ_BtldProMsg(g_BtldCore, msg, ret, type);
			}
			else
			{
				ProcMsg(msg);
			}

		 //mDEBUG( msg);				
		}
	}
}