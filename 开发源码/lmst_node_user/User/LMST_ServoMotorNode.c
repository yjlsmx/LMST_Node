/*
 * LMST_ServoMotorNode.c
 *
 * Created: 2016/9/2 
 *  Author: YJ
 * Email: huangyj@lzrobot.com
 */
 
#include "LMST_ServoMotorNode.h"

#include "cJSON.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "SmartTuna.h"
#include "LMST_TailNode.h"
#include "LMST_NodeCommon.h"

static void ReplyHeatBeat(char *msg)
{
	char *tmp = "{\"c\":\"HeartBeat\",\"n\":0,\"s\":1,\"Servo_l\":%d,\"Servo_r\":%d,\"Motor_l\":%d,\"Motor_r\":%d,\"temp\":%d,\"humidity\":%d}";
	char buf[240];
	YJ_MtctParam p = {0};
	
	YJ_MtctGetParam(g_MtctCore, &p);
	snprintf(buf, 240, tmp, p.LServo, p.RServo, p.LMotor, p.RMotor, 0, 0);
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
				//mERR(msg);
				break;
			}
			else
			{
				s = cJSON_GetObjectItem( json , "c");
				if( s == NULL || s->type != cJSON_String)
				{
					mERR("have not c");
					//mERR(msg);
					break;
				}
				
				v = cJSON_GetObjectItem( json , "value");
				
				if(strcmp(s->valuestring, "SetSteerDirection_l") == 0)
				{
					p.LServo = v->valueint;				
				}
				else if(strcmp(s->valuestring, "SetMotorSpeed_l") == 0)
				{
					p.LMotor = v->valueint;	
				}
				else if(strcmp(s->valuestring, "SetSteerDirection_r") == 0)
				{
					p.RServo = v->valueint;	
				}
				else if(strcmp(s->valuestring, "SetMotorSpeed_r") == 0)
				{
					p.RMotor = v->valueint;	
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
					//mERR(msg);
					break;
				}
				
				YJ_MtctSetParam(g_MtctCore, &p);
				//mDEBUG("c: %d", s->valueint);
			}
			
		}while(0);
	 
		if(json)
		 cJSON_Delete(json);
}



void YJ_ServoMotorNodeTask( void *parameters )
{
		int32_t 	ret = 1; 
		char       * msg = pvPortMalloc(LM_MAX_CAN_PAYLOAD);
	  int32_t   type;
	
	  const portTickType xDelay = pdMS_TO_TICKS(5);  //宏pdMS_TO_TICKS用于将毫秒转成节拍数
	  mDEBUG("boot ----> user");
		while(1)
		{
			vTaskDelay( xDelay );	
			 if((ret = YJ_RecvMsg( msg, LM_MAX_CAN_PAYLOAD, &type)) > 0)
			 {
				 ProcMsg(msg);
				 //mDEBUG( msg);				
			 }
		}
}