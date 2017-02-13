#include "OnChip.h"
u8 isFlashBusy = 0;
u8 FLASH_ProgramBytes(u32 Address, u8 *Buffer, u16 ByteCount)
{
   
    FLASH_Status FLASHStatus = FLASH_COMPLETE;
    u16 i = 0;
    while((i<ByteCount) && (FLASHStatus == FLASH_COMPLETE))
    {
        FLASHStatus = FLASH_ProgramHalfWord(Address, *(u16*)Buffer);
        if(FLASHStatus!=FLASH_COMPLETE)
			return 1;
		i = i+2;
        Address = Address + 2;
        Buffer = Buffer + 2;
    }
	return 0;
}

u8 FLASH_ProgramWords(u32 Address, u32 *Buffer, u16 WordCount)
{
   
    FLASH_Status FLASHStatus = FLASH_COMPLETE;
    u16 i = 0;
    while((i<(WordCount)) && (FLASHStatus == FLASH_COMPLETE))
    {
        FLASHStatus = FLASH_ProgramWord(Address, *Buffer);
		if(FLASHStatus!= FLASH_COMPLETE)				//错误判断
			return 1;
        i = i+1;
        Address = Address + 4;
        Buffer = Buffer + 1;
    }
	return 0;
}

u8 FlashErasePage(u32 PageAddress,u16 PageCount)					//Flash删除多个页
{
	FLASH_Status FLASHStatus = FLASH_COMPLETE;
	u16 i;
	FLASH_Unlock();
	for(i=0;i<PageCount;i++)
	{
		FLASHStatus=FLASH_ErasePage(PageAddress);
		PageAddress+=FLASH_PAGE_SIZE;
		if(FLASHStatus!=FLASH_COMPLETE)
			return 1;
	}
	FLASH_Lock();
	return 0;
}

static u8 g_PageData[FLASH_PAGE_SIZE];
u8 FlashWriteBytes(u32 Address, u8 *Buffer, u16 ByteCount) 
{
	u32 PageAddress;
	u16 PageOffset;

	u16 i;
	volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;
	isFlashBusy = 1;
	FLASH_Unlock();
	PageAddress=Address & (~(FLASH_PAGE_SIZE -1 )) ;	//页地址
    PageOffset=Address & (FLASH_PAGE_SIZE -1); 			//页偏移
	
	FlashReadBytes(PageAddress,g_PageData,FLASH_PAGE_SIZE);
	
	for(i=0;i<ByteCount;i++)
	{
		if(g_PageData[PageOffset+i]!=0xFF)				//如果要写入的部分不为FF则需要擦除
			break;
	}
	if(i<ByteCount)										//判断需要擦除
	{
		FLASHStatus = FLASH_ErasePage(PageAddress);		//擦除页
		if(FLASHStatus!= FLASH_COMPLETE){				//错误判断
			FLASH_Lock();
			isFlashBusy = 0;
			return 1;
		}
		for(i=0;i<ByteCount;i++)						//改变读取值
		{ 
			g_PageData[PageOffset+i]=Buffer[i]; 
		}
		i=FLASH_ProgramBytes(PageAddress,g_PageData,FLASH_PAGE_SIZE);//写入整个扇区 
		if(i==1){
			FLASH_Lock();
			isFlashBusy = 0;
			return 1;
		}
	}	
	else
	{
		i=FLASH_ProgramBytes(Address,Buffer,ByteCount);
		if(i==1){
			FLASH_Lock();
			isFlashBusy = 0;
			return 1;
		}
	}
  FLASH_Lock();
	isFlashBusy = 0;
	return 0;
}

u8 FlashWriteWords(u32 Address, u32 *Buffer, u16 WordCount) 
{
	return FlashWriteBytes(Address, (u8 *)Buffer, WordCount*4);
}

void FlashReadBytes(u32 Address, u8 *Buffer, u16 ByteCount) 
{
    u16 i = 0;
    while(i < ByteCount) 
    {
        *(Buffer + i) = *(__IO u8*) Address++;
        i++;
    }
}

void FlashReadWords(u32 Address, u32 *Buffer, u16 WordCount)
{
	FlashReadBytes(Address, (u8 *)Buffer,(WordCount*4));
}

void SaveParameter(int32_t ServoOffset[3])
{
	Config ConfigSave;
	if(isFlashBusy)return;
	ConfigSave.ServoOffset[0] = ServoOffset[0];
	ConfigSave.ServoOffset[1] = ServoOffset[1];
	ConfigSave.ServoOffset[2] = ServoOffset[2];
	ConfigSave.Version = VERSION;
	FlashWriteWords(ConfigAddress,(u32 *)&ConfigSave,4);
	/*
	PID *PIDRead;
	PIDSave=&Roll;
	//FlashWriteBytes(0x08008000,(u8 *)&PIDSave,4);
	FlashWriteWords(0x08008000,(u32 *)PIDSave,7);
	
	PIDRead=&Pitch;
	//FlashReadBytes(0x08008000,(u8 *)&PIDRead,4);
	FlashReadWords(0x08008000,(u32 *)PIDRead,7);
	PIDRead++;*/
}
void ReadParameter(int32_t ServoOffset[3])
{
	Config ConfigRead;
	
	FlashReadWords(ConfigAddress,(u32 *)&ConfigRead,4);
	if(ConfigRead.Version == VERSION){
		ServoOffset[0] = ConfigRead.ServoOffset[0];
		ServoOffset[1] = ConfigRead.ServoOffset[1];
		ServoOffset[2] = ConfigRead.ServoOffset[2];
	}else{
		ServoOffset[0] = 0;
		ServoOffset[1] = 0;
		ServoOffset[2] = 0;
		SaveParameter(ServoOffset);
	}
}

//设置运行状态 1：用户态 其他值：运行bootloader
void YJ_SetRunMode(int32_t mode)
{
		FlashWriteWords(YJ_RUN_MODE_FLAGS_FLASH_ADDR,(int32_t *)&mode, 1);
}

//获取运行状态
int32_t YJ_GetRunMode()
{
		int32_t mode;
		FlashReadWords(YJ_RUN_MODE_FLAGS_FLASH_ADDR, (u32 *)&mode, 1);
		return mode;
}
		