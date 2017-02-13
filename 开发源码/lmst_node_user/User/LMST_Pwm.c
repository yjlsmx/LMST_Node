/*
 * LMST_Pwm.c
 *
 * Created: 2016/9/2 
 *  Author: YJ
 * Email: huangyj@lzrobot.com
 */
 
#include "LMST_Pwm.h"
#include <stdio.h>
#include <stdlib.h>
#include "stm32f10x.h"
#include "stm32f10x_tim.h" 
#include "misc.h"
#include "CPGCore.h"

static void Timer2Initialize(void)
{
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseStructure;
	TIM_OCInitTypeDef				TIM_OCInitStructure;
	GPIO_InitTypeDef        GPIO_InitStructure;
    
	//关闭Jtag
	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	//四路PWM GPIO 端口重映射
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2,ENABLE);
	
	#if _LMST_NODE_TYPE == LMST_SERVO_MOTOR_NODE
		//电机正反转控制
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_12|GPIO_Pin_13;					
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);	
		GPIO_ResetBits(GPIOB,GPIO_Pin_8|GPIO_Pin_9);
		GPIO_ResetBits(GPIOB,GPIO_Pin_12|GPIO_Pin_13);
	#endif
	
	//PWM GPIO设定
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3|GPIO_Pin_10|GPIO_Pin_11;		//四路PWM输出
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_15;							//四路PWM输出
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//APB定时器2允许
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	//频率设定
	TIM_TimeBaseStructure.TIM_Period = 40000;						//计数上线		20ms
	TIM_TimeBaseStructure.TIM_Prescaler = 36-1;						//pwm时钟分频	2MHZ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;						
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		//向上计数
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	/* PWM1 Mode configuration: Channel */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 3000;								//初始占空比
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	//CH1
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	//CH2
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	//CH3
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	//CH4
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
	//ARR
	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_ClearFlag(TIM2,TIM_FLAG_Update);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	//开启定时器
	TIM_Cmd(TIM2, ENABLE);
	TIM2->CCR1=3000;
	TIM2->CCR2=3000;
	TIM2->CCR3=3000;
	TIM2->CCR4=3000;
	
}

static void RCCInitialize(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
}

static void GPIOInitialize(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;						//LED
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;					
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
    //GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST,ENABLE);
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5|GPIO_Pin_6;					
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);		
}

static void NVICInitialize(void)
{
//	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
    //定时器2中断
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;					//定时器中断2
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//2;			//中断抢占优先级1	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;//2;				//中断响应优先级2
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;					//使能中断
	NVIC_Init(&NVIC_InitStructure);	
	
}

 /*=================================================
        尾舱 设置占空比参数
					return
						1				成功
						0				失败
==================================================*/
int32_t YJ_PwmSet_tail(double Sita)
{

	TIM_ClearITPendingBit(TIM2,TIM_FLAG_Update);
		TIM2->CCR1 = 3000 + Sita;
		TIM2->CCR2 = 3000 + Sita;
		TIM2->CCR3 = 3000 + Sita;
		TIM2->CCR4 = 3000 + Sita;	
		
		return 1;
}

 /*=================================================
        螺旋桨推进舱(ServoMotorNode) 设置占空比参数
					return
						1				成功
						0				失败
==================================================*/
int32_t YJ_PwmSet_sm(	u8 LServo, u8 RServo, u8 LMotor, u8 RMotor)
{
	TIM_ClearITPendingBit(TIM2,TIM_FLAG_Update);
	TIM2->CCR1 = 3000 - ((int)(LServo&0x0F)-7)*200;
	TIM2->CCR2 = 3000 + ((int)(RServo&0x0F)-7)*200;
	TIM2->CCR3 = abs((int)(LMotor&0x0F)-7)*5000;
	TIM2->CCR4 = abs((int)(RMotor&0x0F)-7)*5000;
	
	
	if((LMotor&0x0F)==7)
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_8|GPIO_Pin_9);
	}
	else if((LMotor&0x0F)>7)
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_8);
		GPIO_SetBits(GPIOB,GPIO_Pin_9);
	}
	else if((LMotor&0x0F)<7)
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_9);
		GPIO_SetBits(GPIOB,GPIO_Pin_8);
	}
	
	if((RMotor&0x0F)==7)
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_12|GPIO_Pin_13);
	}
	else if((RMotor&0x0F)<7)
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_12);
		GPIO_SetBits(GPIOB,GPIO_Pin_13);
	}
	else if((RMotor&0x0F)>7)
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_13);
		GPIO_SetBits(GPIOB,GPIO_Pin_12);
	}
	
	return 1;
}
/*=================================================
        pwm模块初始化
==================================================*/
 void YJ_PwmInit()
 {
		RCCInitialize();                //RCC初始化
		GPIOInitialize();
		Timer2Initialize();	
		//NVICInitialize();
 }
 
 /*=================================================
        pwm模块反初始化
==================================================*/
 void YJ_PwmUnInit()
 {
		TIM_Cmd(TIM2, DISABLE);
 }
