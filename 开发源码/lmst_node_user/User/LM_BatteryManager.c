 /*
 * LM_BatteryManager.c
 *
 * Created: 2016/9/2 
 *  Author: YJ
 * Email: huangyj@lzrobot.com
 */
 
#include "stm32f10x_adc.h"
#include "LM_BatteryManager.h"	
#include "stm32f10x_dma.h"
#include "SmartTuna.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "stm32f10x.h"	   
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"	 
#include "misc.h"		

#include  <stdarg.h>

#define ADC1_DR_Address    ((u32)0x4001244C)	 
static vu16 ADC_ConvertedValue;

static void InitAdc()
{
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	//RCC
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
	
		//设置AD模拟输入端口为输入 1路AD 规则通道
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;//GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/* Enable DMA clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/* Enable ADC1 and GPIOC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , ENABLE);

  	/* DMA channel1 configuration ----------------------------------------------*/
	//使能DMA
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;			            //DMA通道1的地址 
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;	            //DMA传送地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;					            //传送方向
	DMA_InitStructure.DMA_BufferSize = 1;								            //传送内存大小，1个16位
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	 
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;				            //传送内存地址递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;		//ADC1转换的数据是16位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;				//传送的目的地址是16位宽度
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;									//循环
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    
	/* 允许DMA1通道1传输结束中断 */
	//DMA_ITConfig(DMA1_Channel1,DMA_IT_TC, ENABLE);

	//使能DMA通道1
	DMA_Cmd(DMA1_Channel1, ENABLE); 
  
	//ADC配置
	/* ADC转换时间： ─ STM32F103xx增强型产品：时钟为56MHz时为1μs(时钟为72MHz为1.17μs)
	ADC采样范围0-3.3V    */
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);                   //设置ADC的时钟为72MHZ/6=12M 

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC1工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;		//模数转换工作在扫描模式（多通道）还是单次（单通道）模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//模数转换工作在连续模式，还是单次模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;               //规定了顺序进行规则转换的ADC通道的数目。这个数目的取值范围是1到16
	ADC_Init(ADC1, &ADC_InitStructure);
	
	/* ADC1 regular channels configuration [规则模式通道配置]*/ 

	//ADC1 规则通道配置
  	//ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_55Cycles5);	  //通道1采样时间 55.5周期
  ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_55Cycles5);	
		
	//使能ADC1 DMA 
	ADC_DMACmd(ADC1, ENABLE);
	//使能ADC1
	ADC_Cmd(ADC1, ENABLE);	
	
	// 初始化ADC1校准寄存器
	ADC_ResetCalibration(ADC1);
	//检测ADC1校准寄存器初始化是否完成
	while(ADC_GetResetCalibrationStatus(ADC1));
	
	//开始校准ADC1
	ADC_StartCalibration(ADC1);
	//检测是否完成校准
	while(ADC_GetCalibrationStatus(ADC1));
	
	//ADC1转换启动
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	 
	
}

static void UnInitAdc()
{
		ADC_SoftwareStartConvCmd(ADC1, DISABLE);	
		ADC_Cmd(ADC1, DISABLE);	
	  ADC_DMACmd(ADC1, DISABLE);
	  DMA_Cmd(DMA1_Channel1, DISABLE); 
}
	

static void InitLed()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;				//默认OD门复用输出，需上拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;				//GPIO速度10MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	//GPIO_ResetBits(GPIOB,GPIO_Pin_5|GPIO_Pin_6);
	GPIO_ResetBits(GPIOB,GPIO_Pin_5|GPIO_Pin_6);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;				//默认OD门复用输出，需上拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;				//GPIO速度10MHz
	GPIO_Init(GPIOC, &GPIO_InitStructure); 
	//GPIO_SetBits(GPIOA,GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
	GPIO_ResetBits(GPIOC,GPIO_Pin_13);
}

static void BtmgTask( void *parameters )
{
	int32_t 	ret = 1; 

	const portTickType xDelay = pdMS_TO_TICKS(2000);  //宏pdMS_TO_TICKS用于将毫秒转成节拍数
	
	ret = YJ_BtmgGetParam();
	while(1)
	{
		ret = (ret + YJ_BtmgGetParam()) / 2;
		
		if(ret > 7500)
		{
			GPIO_SetBits(GPIOC,GPIO_Pin_13);
			GPIO_SetBits(GPIOB,GPIO_Pin_5);
			GPIO_SetBits(GPIOB,GPIO_Pin_6);
		}
		else if(ret > 6500)
		{
			GPIO_ResetBits(GPIOC,GPIO_Pin_13);
			GPIO_SetBits(GPIOB,GPIO_Pin_6);
			GPIO_SetBits(GPIOB,GPIO_Pin_5);

		}
		else
		{
			GPIO_ResetBits(GPIOC,GPIO_Pin_13);
			GPIO_ResetBits(GPIOB,GPIO_Pin_6);
			GPIO_SetBits(GPIOB,GPIO_Pin_5);
		}
		
		vTaskDelay( xDelay );	
	}
}

/*=================================================
        获取电源参数（电池电压）
==================================================*/
int32_t YJ_BtmgGetParam()
{
	return (ADC_ConvertedValue * 3300 * 14.7 / (4096 * 4.7));
}

/*=================================================
        反初始化电源管理模块
==================================================*/
void YJ_BtmgUnInit()
{
	UnInitAdc();
}

/*=================================================
        初始化电源管理模块
==================================================*/
void YJ_BtmgInit()
{
	InitAdc();
	InitLed();
	
	xTaskCreate( BtmgTask, ( signed portCHAR * ) "BtmgTask", 
	configMINIMAL_STACK_SIZE, NULL, 
		LMST_BTMG_TASK_PRIORITY, NULL);
}




