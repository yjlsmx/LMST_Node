 /*
 * LMST_SensorNode.c
 *
 * Created: 2016/9/2 
 *  Author: YJ
 * Email: huangyj@lzrobot.com
 */
 
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "cJSON.h"
#include "SmartTuna.h"
#include "LMST_NodeCommon.h"
#include "LM_Mpu.h"
#include "LM_Common.h"
#include "LMST_SensorNode.h"

static void ReplyHeatBeat(char *msg)
{
	YJ_MpuParam p;
	char *tmp = "{\"c\":\"HeartBeat\",\"n\":0,\"s\":1,\"temp\":%d,\"accel_x\":%d,\"accel_y\":%d,\"accel_z\":%d,\"gyro_x\":%d,\"gyro_y\":%d,\"gyro_z\":%d}";
	char buf[240];
	
	YJ_MpuGetParam(g_MpuCore, &p);
	snprintf(buf, 240, tmp, p.Temp, p.Accel_X, p.Accel_Y, p.Accel_Z, p.Gyro_X, p.Gyro_Y, p.Gyro_Z);
	YJ_SendMsg(buf, 3);
}
		
static void ProcMsg(char *msg)
{
		cJSON *json = NULL , *s = NULL;						
	 do{
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


void YJ_SensorNodeTask( void *parameters )
{
		int32_t 	ret = 1; 
		char       * msg = pvPortMalloc(LM_MAX_CAN_PAYLOAD);
		int32_t   type;
		uint64_t    timeout;
		YJ_MpuParam p = {0};
	
	  const portTickType xDelay = pdMS_TO_TICKS(5);  
	  mDEBUG("boot ----> user");
	while(1)
	{
		vTaskDelay( xDelay );
			YJ_MpuGetParam(g_MpuCore, &p);
		if((ret = YJ_RecvMsg( msg, LM_MAX_CAN_PAYLOAD, &type)) > 0)
		{
			if(type == 6 && type == 8)
			{
			}
			else
			{
				ProcMsg(msg);
			}

		 //mDEBUG( msg);				
		}
	}
}
