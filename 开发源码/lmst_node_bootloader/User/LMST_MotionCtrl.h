#pragma once 
#include "stm32f10x.h"

typedef struct YJ_MtctCore YJ_MtctCore;

typedef struct
{
	//尾舱参数
	u8 Speed;
	u8 Direct;
	u8 Mode;
	int32_t ServoOffset[3];
	
	//螺旋桨推进舱参数
	u8 LServo;
	u8 RServo;
	u8 LMotor;
	u8 RMotor;
	
}YJ_MtctParam;

/*=================================================
        运动控制初始化
    return
        句柄           成功
        NULL           失败
==================================================*/
YJ_MtctCore * YJ_MtctInit(void);

/*=================================================
        运动控制反初始化
==================================================*/
void YJ_MtctUnInit(YJ_MtctCore *h);

/*=================================================
        设置控制参数
    return
        1           成功
        0           失败
==================================================*/
int32_t YJ_MtctSetParam(YJ_MtctCore *h, YJ_MtctParam * p);

/*=================================================
        获取控制参数
    return
        1           成功
        0           失败
==================================================*/
int32_t YJ_MtctGetParam(YJ_MtctCore *h, YJ_MtctParam * p);

/*=================================================
        把尾鳍的偏移量保存到flash里
==================================================*/
void YJ_SaveTraiFinOffset(YJ_MtctCore *h);


