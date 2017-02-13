#ifndef OnChipH
#define OnChipH
#include "SmartTuna.h"
#define ConfigAddress 0x0800fC00
#define YJ_RUN_MODE_FLAGS_FLASH_ADDR		(ConfigAddress + 4 * sizeof(int32_t)) 
#define YJ_USER_PROGRAM_FLASH_ADDR			0x08008000
#define VERSION 1
#if defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || defined (STM32F10X_CL) || defined (STM32F10X_XL)
#define FLASH_PAGE_SIZE ((uint16_t)0x800)
#else
#define FLASH_PAGE_SIZE ((uint16_t)0x400)
#endif
u8 FlashErasePage(u32 PageAddress,u16 PageCount);
extern u8 FlashWriteBytes(u32 Address, u8 *Buffer, u16 ByteCount);
extern void FlashReadBytes(u32 Address, u8 *Buffer, u16 ByteCount);
extern void FlashReadWords(u32 Address, u32 *Buffer, u16 WordCount);
void SaveParameter(int32_t ServoOffset[3]);
extern void ReadParameter(int32_t ServoOffset[3]);
typedef struct Parameter
{
	int32_t ServoOffset[3];
	int32_t Version;
}Config;

//设置系统运行状态
void YJ_SetRunMode(int32_t mode);

//获取系统运行状态
int32_t YJ_GetRunMode();

void testFlash();

#endif
