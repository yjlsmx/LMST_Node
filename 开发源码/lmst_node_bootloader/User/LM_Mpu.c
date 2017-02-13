 /*
 * LM_Mpu.c
 *
 * Created: 2016/9/2 
 *  Author: YJ
 * Email: huangyj@lzrobot.com
 */
 
#include "LM_Mpu.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "cJSON.h"
#include "SmartTuna.h"
#include "LMST_NodeCommon.h"


/*******************************************************************************
// GY-521 MPU6050 IIC测试程序
// 使用单片机STM32F103C8T6
// 晶振：8.00M
// 编译环境 Keil uVision4
// 时间：2012年5月1日
// 与模块连接 GPIOB6->SCL GPIOB7->SDA      
// QQ：531389319
*******************************************************************************/

#include <stm32f10x.h>
#include <stm32f10x_flash.h> 
#include <stm32f10x_wwdg.h> 
#include  <math.h>    //Keil library  

GPIO_InitTypeDef GPIO_InitStructure;
ErrorStatus HSEStartUpStatus;

#define   uchar unsigned char
#define   uint unsigned int	

// 定义MPU6050内部地址
//****************************************
#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		0x75	//IIC地址寄存器(默认数值0x68，只读)


//****************************

#define	MPU6050_Addr   0xD0	  //定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改

static unsigned char TX_DATA[4];  	 //显示据缓存区
static unsigned char BUF[20];       //接收数据缓存区
static char  test=0; 				 //IIC用到
static short T_X,T_Y,T_Z,T_T;		 //陀螺仪X,Y,Z轴，温度
static short A_X,A_Y,A_Z;		 //陀螺仪X,Y,Z轴，温度

//************************************
/*模拟IIC端口输出输入定义*/
#define SCL_H         GPIOB->BSRR = GPIO_Pin_6
#define SCL_L         GPIOB->BRR  = GPIO_Pin_6 
   
#define SDA_H         GPIOB->BSRR = GPIO_Pin_7
#define SDA_L         GPIOB->BRR  = GPIO_Pin_7

#define SCL_read      GPIOB->IDR  & GPIO_Pin_6
#define SDA_read      GPIOB->IDR  & GPIO_Pin_7

/* 函数申明 -----------------------------------------------*/
void RCC_Configuration(void);
//void GPIO_Configuration(void);
//void NVIC_Configuration(void);
void Delay(u32 nTime);
void Delayms(vu32 m);  
/* 变量定义 ----------------------------------------------*/

  /*******************************/
static void DATA_printf(uchar *s,short temp_data)
{
	if(temp_data<0){
	temp_data=-temp_data;
    *s='-';
	}
	else *s=' ';
    *++s =temp_data/100+0x30;
    temp_data=temp_data%100;     //取余运算
    *++s =temp_data/10+0x30;
    temp_data=temp_data%10;      //取余运算
    *++s =temp_data+0x30; 	
}

/*******************************************************************************
* Function Name  : I2C_GPIO_Config
* Description    : Configration Simulation IIC GPIO
* Input          : None 
* Output         : None
* Return         : None
****************************************************************************** */
static void I2C_GPIO_Config(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure; 
 
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/*******************************************************************************
* Function Name  : I2C_delay
* Description    : Simulation IIC Timing series delay
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
static void I2C_delay(void)
{
		
   u8 i=30; //这里可以优化速度	，经测试最低到5还能写入
   while(i) 
   { 
     i--; 
   }  
}

static void delay5ms(void)
{
		
   int i=5000;  
   while(i) 
   { 
     i--; 
   }  
}
/*******************************************************************************
* Function Name  : I2C_Start
* Description    : Master Start Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : Wheather	 Start
****************************************************************************** */
static bool I2C_Start(void)
{
	SDA_H;
	SCL_H;
	I2C_delay();
	if(!SDA_read)return FALSE;	//SDA线为低电平则总线忙,退出
	SDA_L;
	I2C_delay();
	if(SDA_read) return FALSE;	//SDA线为高电平则总线出错,退出
	SDA_L;
	I2C_delay();
	return TRUE;
}
/*******************************************************************************
* Function Name  : I2C_Stop
* Description    : Master Stop Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
static void I2C_Stop(void)
{
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SDA_H;
	I2C_delay();
} 
/*******************************************************************************
* Function Name  : I2C_Ack
* Description    : Master Send Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
static void I2C_Ack(void)
{	
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
}   
/*******************************************************************************
* Function Name  : I2C_NoAck
* Description    : Master Send No Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
static void I2C_NoAck(void)
{	
	SCL_L;
	I2C_delay();
	SDA_H;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
} 
/*******************************************************************************
* Function Name  : I2C_WaitAck
* Description    : Master Reserive Slave Acknowledge Single
* Input          : None
* Output         : None
* Return         : Wheather	 Reserive Slave Acknowledge Single
****************************************************************************** */
static bool I2C_WaitAck(void) 	 //返回为:=1有ACK,=0无ACK
{
	SCL_L;
	I2C_delay();
	SDA_H;			
	I2C_delay();
	SCL_H;
	I2C_delay();
	if(SDA_read)
	{
      SCL_L;
	  I2C_delay();
      return FALSE;
	}
	SCL_L;
	I2C_delay();
	return TRUE;
}
/*******************************************************************************
* Function Name  : I2C_SendByte
* Description    : Master Send a Byte to Slave
* Input          : Will Send Date
* Output         : None
* Return         : None
****************************************************************************** */
static void I2C_SendByte(u8 SendByte) //数据从高位到低位//
{
    u8 i=8;
    while(i--)
    {
        SCL_L;
        I2C_delay();
      if(SendByte&0x80)
        SDA_H;  
      else 
        SDA_L;   
        SendByte<<=1;
        I2C_delay();
		SCL_H;
        I2C_delay();
    }
    SCL_L;
}  
/*******************************************************************************
* Function Name  : I2C_RadeByte
* Description    : Master Reserive a Byte From Slave
* Input          : None
* Output         : None
* Return         : Date From Slave 
****************************************************************************** */
static unsigned char I2C_RadeByte(void)  //数据从高位到低位//
{ 
    u8 i=8;
    u8 ReceiveByte=0;

    SDA_H;				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L;
      I2C_delay();
	  SCL_H;
      I2C_delay();	
      if(SDA_read)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L;
    return ReceiveByte;
} 
//ZRX          
//单字节写入*******************************************

static bool Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)		     //void
{
  	if(!I2C_Start())return FALSE;
    I2C_SendByte(SlaveAddress);   //发送设备地址+写信号//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//设置高起始地址+器件地址 
    if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}
    I2C_SendByte(REG_Address );   //设置低起始地址      
    I2C_WaitAck();	
    I2C_SendByte(REG_data);
    I2C_WaitAck();   
    I2C_Stop(); 
    delay5ms();
    return TRUE;
}

//单字节读取*****************************************
static unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address)
{   unsigned char REG_data;     	
	if(!I2C_Start())return FALSE;
    I2C_SendByte(SlaveAddress); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//设置高起始地址+器件地址 
    if(!I2C_WaitAck()){I2C_Stop();test=1; return FALSE;}
    I2C_SendByte((u8) REG_Address);   //设置低起始地址      
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(SlaveAddress+1);
    I2C_WaitAck();

	REG_data= I2C_RadeByte();
    I2C_NoAck();
    I2C_Stop();
    //return TRUE;
	return REG_data;

}						      

/*
********************************************************************************
** 函数名称 ： RCC_Configuration(void)
** 函数功能 ： 时钟初始化
** 输    入	： 无
** 输    出	： 无
** 返    回	： 无
********************************************************************************
*/
static void RCC_Configuration(void)
{   
   /* Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG and AFIO clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
}

/*
********************************************************************************
** 函数名称 ： Delay(vu32 nCount)
** 函数功能 ： 延时函数
** 输    入	： 无
** 输    出	： 无
** 返    回	： 无
********************************************************************************
*/
 void Delay(vu32 nCount)
{
  for(; nCount != 0; nCount--);
}

/*
********************************************************************************
** 函数名称 ： void Delayms(vu32 m)
** 函数功能 ： 长延时函数	 m=1,延时1ms
** 输    入	： 无
** 输    出	： 无
** 返    回	： 无
********************************************************************************
*/
static  void Delayms(vu32 m)
{
  u32 i;
  
  for(; m != 0; m--)	
       for (i=0; i<50000; i++);
}

//初始化MPU6050，根据需要请参考pdf进行修改************************
static void Init_MPU6050(void)
{
/*
   Single_Write(MPU6050_Addr,PWR_M, 0x80);   //
   Single_Write(MPU6050_Addr,SMPL, 0x07);    //
   Single_Write(MPU6050_Addr,DLPF, 0x1E);    //±2000°
   Single_Write(MPU6050_Addr,INT_C, 0x00 );  //
   Single_Write(MPU6050_Addr,PWR_M, 0x00);   //
*/
  Single_Write(MPU6050_Addr,PWR_MGMT_1, 0x00);	//解除休眠状态
	Single_Write(MPU6050_Addr,SMPLRT_DIV, 0x07);
	Single_Write(MPU6050_Addr,CONFIG, 0x06);
	Single_Write(MPU6050_Addr,GYRO_CONFIG, 0x18);
	Single_Write(MPU6050_Addr,ACCEL_CONFIG, 0x01);
}
	
//******读取MPU6050数据****************************************
static void READ_MPU6050(void)
{
   BUF[0]=Single_Read(MPU6050_Addr,GYRO_XOUT_L); 
   BUF[1]=Single_Read(MPU6050_Addr,GYRO_XOUT_H);
   T_X=	(BUF[1]<<8)|BUF[0];
   T_X/=16.4; 						   //读取计算X轴数据

   BUF[2]=Single_Read(MPU6050_Addr,GYRO_YOUT_L);
   BUF[3]=Single_Read(MPU6050_Addr,GYRO_YOUT_H);
   T_Y=	(BUF[3]<<8)|BUF[2];
   T_Y/=16.4; 						   //读取计算Y轴数据
   BUF[4]=Single_Read(MPU6050_Addr,GYRO_ZOUT_L);
   BUF[5]=Single_Read(MPU6050_Addr,GYRO_ZOUT_H);
   T_Z=	(BUF[5]<<8)|BUF[4];
   T_Z/=16.4; 					       //读取计算Z轴数据

	BUF[6]=Single_Read(MPU6050_Addr,TEMP_OUT_L); 
  BUF[7]=Single_Read(MPU6050_Addr,TEMP_OUT_H); 
  T_T=(BUF[7]<<8)|BUF[6];
  T_T = 35+ ((double) (T_T + 13200)) / 280;// 读取计算出温度
	
	 BUF[8]=Single_Read(MPU6050_Addr,ACCEL_XOUT_L); 
   BUF[9]=Single_Read(MPU6050_Addr,ACCEL_XOUT_H);
   A_X=	(BUF[9]<<8)|BUF[8];
   A_X/=16.4; 						   //读取计算X轴数据
   BUF[10]=Single_Read(MPU6050_Addr,ACCEL_YOUT_L);
   BUF[11]=Single_Read(MPU6050_Addr,ACCEL_YOUT_H);
   A_Y=	(BUF[11]<<8)|BUF[10];
   A_Y/=16.4; 						   //读取计算Y轴数据
   BUF[12]=Single_Read(MPU6050_Addr,ACCEL_ZOUT_L);
   BUF[13]=Single_Read(MPU6050_Addr,ACCEL_ZOUT_H);
   A_Z=	(BUF[13]<<8)|BUF[12];
   A_Z/=16.4; 					       //读取计算Z轴数据
	 
}

struct YJ_MpuCore
{
		int32_t 	IsInit;		
};

/* =========================================================
		反初始化MPU模块
===========================================================*/
void YJ_MpuUnInit(YJ_MpuCore * h)
{
	h->IsInit = 0;
	return;
}

/* =========================================================
		初始化MPU模块
			return
				句柄	成功
				NULL  失败
===========================================================*/
YJ_MpuCore * YJ_MpuInit(void)
{
	YJ_MpuCore * h = (YJ_MpuCore *)pvPortMalloc(sizeof(struct YJ_MpuCore));
	
	if(h == NULL)
		return NULL;
	
	memset(h, 0, sizeof(struct YJ_MpuCore));
	
	RCC_Configuration();		 //配置RCC
  I2C_GPIO_Config();		 //配置IIC使用端口
  Delayms(10);				 //延时
  Init_MPU6050();		     //初始化MPU6050
	
	h->IsInit = 1;
}

/* =========================================================
	   获取MPU数据
			return
				1	 成功
				0  失败
===========================================================*/
int32_t YJ_MpuGetParam(YJ_MpuCore *h, YJ_MpuParam *p)
{
	if(h == NULL || !h->IsInit)
		return 0;
	
	vTaskSuspendAll();
	READ_MPU6050();	
	p->Accel_X = A_X;
	p->Accel_Y = A_Y;
	p->Accel_Z = A_Z;
	p->Gyro_X  = T_X;
	p->Gyro_Y  = T_Y;
	p->Gyro_Z  = T_Z;
	p->Temp		 = T_T;
	xTaskResumeAll();
	
	return 1;
}
