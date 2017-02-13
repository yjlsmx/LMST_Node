#pragma once
#include <stdint.h>


typedef struct YJ_MpuCore YJ_MpuCore;

typedef struct YJ_MpuParam 
{
	int16_t Accel_X;
	int16_t Accel_Y;
	int16_t Accel_Z;
	
	int16_t Gyro_X;
	int16_t Gyro_Y;
	int16_t Gyro_Z;
	
	int32_t Temp;
}YJ_MpuParam;

/* =========================================================
		反初始化MPU模块
===========================================================*/
void YJ_MpuUnInit(YJ_MpuCore * h);

/* =========================================================
		初始化MPU模块
			return
				句柄	成功
				NULL  失败
===========================================================*/
YJ_MpuCore * YJ_MpuInit(void);

/* =========================================================
	   获取MPU数据
			return
				1	 成功
				0  失败
===========================================================*/
int32_t YJ_MpuGetParam(YJ_MpuCore *h, YJ_MpuParam *p);

