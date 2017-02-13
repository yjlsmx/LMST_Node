#pragma once
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h" 
 #ifdef __cplusplus
extern "C" {
#endif
//#include <pthread.h>

typedef struct HiQueueRecord  
{  
    int					Capacity;  
    int					head;  
    int					tail;  
    int					Size;  
    unsigned char		*Array;  
	//pthread_mutex_t		hMutex;
		SemaphoreHandle_t mutex; 	
}HiQueueRecord, *HiQueue;


/*构造一个空队列*/  
HiQueue HiCreateQueue(char *buff, int32_t bufflen);  
/*销毁一个队列*/  
void  HiDestroyQueue(HiQueue q);

/* =============================================
		查询队列空闲的空间
		return
			空闲空间
================================================*/
int	HiGetQueueIdleNum(HiQueue q);

/* =============================================
		入列
		return
			1		成功
			-1		失败 
================================================*/
int	HiEnQueue(char * buf, int len, HiQueue q);

/* =============================================
		入列（无锁）
		return
			1		成功
			-1		失败
================================================*/
int	HiEnQueue_nolock(char * buf, int len, HiQueue q);
/* =============================================
			清空队列
================================================*/
void HiCleanQueue( HiQueue q);

/* =============================================
			删除队列的部分字节
				return
					1		成功
					-1		失败
================================================*/
int HiDeleQueueData(int len,  HiQueue q);

/* =============================================
		出列
		return
			1		成功
			-1		失败 长度不足
================================================*/
int	HiDeQueue(char * buf, int len, HiQueue q);

/* =============================================
		出列(无锁)
		return
			1		成功
			-1		失败 长度不足
================================================*/
int	HiDeQueue_nolock(char * buf, int len, HiQueue q);
/* =============================================
		窥视
		return
			1		成功
			-1		失败 长度不足

		int	offset 偏移offset个字节开始peak
================================================*/
int	HiPeakQueue(char * buf, int len, int offset, HiQueue q);

/* =============================================
		查询队列数据长度
		return
			数据长度
================================================*/
int	HiGetQueueDataLen(HiQueue q);

/* =============================================
		队列数据循环右移

================================================*/
int	HiQueueRR(int32_t byte, HiQueue q);

#ifdef __cplusplus
}
#endif
