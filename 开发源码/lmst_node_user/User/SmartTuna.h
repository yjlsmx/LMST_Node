#ifndef SMARTTUNAH
#define SMARTTUNAH
#include "stdlib.h"

#include "stm32f10x.h"
//#include "stm32f10x_adc.h"
//#include "stm32f10x_bkp.h"
//#include "stm32f10x_can.h"
//#include "stm32f10x_crc.h"
//#include "stm32f10x_dac.h"
//#include "stm32f10x_dbgmcu.h"
//#include "stm32f10x_dma.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_flash.h"
//#include "stm32f10x_fsmc.h"
#include "stm32f10x_gpio.h"
//#include "stm32f10x_i2c.h"
//#include "stm32f10x_iwdg.h"
//#include "stm32f10x_pwr.h"
#include "stm32f10x_rcc.h"
//#include "stm32f10x_rtc.h"
//#include "stm32f10x_sdio.h"
//#include "stm32f10x_spi.h"
#include "stm32f10x_tim.h" 
#include "stm32f10x_usart.h"
//#include "stm32f10x_wwdg.h"
#include "misc.h"
#include "CPGCore.h"

#include "LMST_MotionCtrl.h"
#include "LM_Cudp_m3.h"
#include "LM_Uudp.h"
#include "LM_SystemClock.h"
#include "LM_Mpu.h"
#include "LM_BuoyantDevices.h"

#define LMST_SYSTEM_CLOCK_TASK_PRIORITY			20 //系统时钟有最高的优先级
#define LMST_WDG_TASK_PRIORITY 						  10
#define LMST_MTCT_TASK_PRIORITY 						10
#define LMST_NODE_TASK_PRIORITY 						10
#define LMST_BTMG_TASK_PRIORITY 						10
#define LMST_BTLD_TASK_PRIORITY 						10
#define LMST_BUTD_TASK_PRIORITY 						10
#define LMST_CUDP_TASK_PRIORITY 						11 
#define LMST_UUDP_TASK_PRIORITY 						12


#define LMST_HEAD_NODE											0xe0
#define LMST_TAIL_NODE											0xf0
#define LMST_SERVO_MOTOR_NODE								0x10
#define LMST_SENSOR_NODE										0x20
#define LMST_HEAD_LOCAL_CTRL_NODE						0xe1
#define LMST_DIVING_NODE										0x30

#define _LMST_NODE_TYPE	 LM_LOCAL_CAN_ADDR

//重启
void YJ_ReBoot(void);

//系统反初始化
void YJ_UnInitSystem(void);

extern YJ_MtctCore 			* g_MtctCore;
extern YJ_CudpCore 			* g_CudpCore;
extern YJ_UudpCore 			* g_UudpCore;
extern YJ_SysClockCore 	* g_SysClockCore;
extern YJ_MpuCore 			* g_MpuCore;
extern YJ_ButdCore 			* g_ButdCore;

//extern const int32_t LMST_NODE_TYPE;

#endif
