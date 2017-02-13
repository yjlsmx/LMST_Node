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
		����ʼ��MPUģ��
===========================================================*/
void YJ_MpuUnInit(YJ_MpuCore * h);

/* =========================================================
		��ʼ��MPUģ��
			return
				���	�ɹ�
				NULL  ʧ��
===========================================================*/
YJ_MpuCore * YJ_MpuInit(void);

/* =========================================================
	   ��ȡMPU����
			return
				1	 �ɹ�
				0  ʧ��
===========================================================*/
int32_t YJ_MpuGetParam(YJ_MpuCore *h, YJ_MpuParam *p);

